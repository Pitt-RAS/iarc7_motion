#!/usr/bin/env python
import rospy
import tf2_ros

from geometry_msgs.msg import TwistStamped
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import TwistStamped

from .abstract_task import AbstractTask
from iarc_tasks.task_states import (TaskRunning,
                                    TaskDone,
                                    TaskCanceled,
                                    TaskAborted,
                                    TaskFailed)
from iarc_tasks.task_commands import (VelocityCommand,
                                      NopCommand,
                                      GroundInteractionCommand,
                                      ConfigurePassthroughMode,
                                      AngleThrottleCommand)

class HeightRecoveryTaskState:
    init = 0
    recover = 1
    done = 2
    failed = 3

class HeightRecoveryTask(AbstractTask):

    def __init__(self, task_request):
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)
        self._canceled = False

        self._transition = None

        try:
            self._TAKEOFF_VELOCITY = rospy.get_param('~takeoff_velocity')
            MIN_MAN_HEIGHT = rospy.get_param('~min_maneuver_height')
            HEIGHT_OFFSET = rospy.get_param('~recover_height_offset')
            self._DELAY_BEFORE_TAKEOFF = rospy.get_param('~delay_before_takeoff')
            self._TRANSFORM_TIMEOUT = rospy.get_param('~transform_timeout')
        except KeyError as e:
            rospy.logerr('Could not lookup a parameter for HeightRecoveryTask')
            raise

        self._RECOVERY_HEIGHT = MIN_MAN_HEIGHT + HEIGHT_OFFSET

        self._state = HeightRecoveryTaskState.init

    def get_desired_command(self):
        if self._canceled:
            return (TaskCanceled(),)

        elif self._state == HeightRecoveryTaskState.init:
            self._state = HeightRecoveryTaskState.recover
            return (TaskRunning(),)

        elif (self._state == HeightRecoveryTaskState.recover):
            try:
                transStamped = self._tf_buffer.lookup_transform(
                                    'map',
                                    'base_footprint',
                                    rospy.Time.now(),
                                    rospy.Duration(self._TRANSFORM_TIMEOUT))
            except (tf2_ros.LookupException,
                    tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException) as ex:
                msg = 'Exception when looking up transform during height recovery'
                rospy.logerr('HeightRecoveryTask: {}'.format(msg))
                rospy.logerr(ex.message)
                return (TaskAborted(msg=msg),)

            # Check if we reached the target height
            if (transStamped.transform.translation.z > self._RECOVERY_HEIGHT):
                self._state = HeightRecoveryTaskState.done
                return (TaskDone(), NopCommand())

            else:
                velocity = TwistStamped()
                velocity.header.frame_id = 'level_quad'
                velocity.header.stamp = rospy.Time.now()
                velocity.twist.linear.z = self._TAKEOFF_VELOCITY
                return (TaskRunning(), VelocityCommand(velocity))

        elif self._state == HeightRecoveryTaskState.done:
            return (TaskDone(), NopCommand())

        elif self._state == HeightRecoveryTaskState.failed:
            return (TaskFailed(msg='HeightRecoveryTask experienced failure'),)

        else:
            return (TaskAborted(msg='Impossible state in HeightRecoveryTask reached'),)

    def cancel(self):
        rospy.loginfo('HeightRecoveryTask canceled')
        self._canceled = True
    
    def send_transition(self, transition):
        self._transition = transition

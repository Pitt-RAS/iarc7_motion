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
                                      AngleThrottleCommand)

class HeightRecoveryTaskState(object):
    init = 0
    recover = 1
    done = 2
    failed = 3

class HeightRecoveryTask(AbstractTask):

    def __init__(self, task_request):
        super(HeightRecoveryTask, self).__init__()

        self._canceled = False
        self._above_min_man_height = False

        self._transition = None

        try:
            self._TAKEOFF_VELOCITY = rospy.get_param('~takeoff_velocity')
            self._MIN_MAN_HEIGHT = rospy.get_param('~min_maneuver_height')
            HEIGHT_OFFSET = rospy.get_param('~recover_height_offset')
            self._DELAY_BEFORE_TAKEOFF = rospy.get_param('~delay_before_takeoff')
            self._TRANSFORM_TIMEOUT = rospy.get_param('~transform_timeout')
        except KeyError as e:
            rospy.logerr('Could not lookup a parameter for HeightRecoveryTask')
            raise

        self._RECOVERY_HEIGHT = self._MIN_MAN_HEIGHT + HEIGHT_OFFSET

        self._state = HeightRecoveryTaskState.init

    def get_desired_command(self):
        if self._canceled:
            return (TaskCanceled(),)

        if self._state == HeightRecoveryTaskState.init:
            self._state = HeightRecoveryTaskState.recover

        if (self._state == HeightRecoveryTaskState.recover):
            try:
                transStamped = self.topic_buffer.get_tf_buffer().lookup_transform(
                                    'map',
                                    'base_footprint',
                                    rospy.Time(0),
                                    rospy.Duration(self._TRANSFORM_TIMEOUT))
            except (tf2_ros.LookupException,
                    tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException) as ex:
                msg = 'Exception when looking up transform during height recovery'
                rospy.logerr('HeightRecoveryTask: {}'.format(msg))
                rospy.logerr(ex.message)
                return (TaskAborted(msg=msg),)

             # Check if we are above minimum maneuver height
            self._above_min_man_height = (transStamped.transform.translation.z
                                            > self._MIN_MAN_HEIGHT)

            # Check if we reached the target height
            if (transStamped.transform.translation.z > self._RECOVERY_HEIGHT):
                self._state = HeightRecoveryTaskState.done
            else:
                velocity = TwistStamped()
                velocity.header.frame_id = 'level_quad'
                velocity.header.stamp = rospy.Time.now()
                velocity.twist.linear.z = self._TAKEOFF_VELOCITY
                return (TaskRunning(), VelocityCommand(velocity))

        if self._state == HeightRecoveryTaskState.done:
            return (TaskDone(), NopCommand())

        if self._state == HeightRecoveryTaskState.failed:
            return (TaskFailed(msg='HeightRecoveryTask experienced failure'),)

        return (TaskAborted(msg='Impossible state in HeightRecoveryTask reached'),)

    def cancel(self):
        rospy.loginfo('HeightRecoveryTask cancellation attempted')
        if self._above_min_man_height:
            rospy.loginfo('HeightRecoveryTask cancellation accepted')
            self._canceled = True
            return True
        else: 
            rospy.loginfo('HeightRecoveryTask cancellation rejected')
            return False

    def set_incoming_transition(self, transition):
        self._transition = transition

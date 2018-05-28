#!/usr/bin/env python
import rospy
import tf2_ros

from geometry_msgs.msg import TwistStamped
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import TwistStamped

from iarc7_msgs.msg import FlightControllerStatus

from .abstract_task import AbstractTask
from iarc_tasks.task_states import (TaskRunning,
                                    TaskDone,
                                    TaskCanceled,
                                    TaskAborted,
                                    TaskFailed)
from iarc_tasks.task_commands import (VelocityCommand,
                                      NopCommand,
                                      GroundInteractionCommand,
                                      AngleThrottleCommand,
                                      ResetLinearProfileCommand)

class TakeoffTaskState(object):
    init = 0
    takeoff = 1
    ascend = 2
    stabilize = 3
    done = 4
    failed = 5

class TakeoffTask(AbstractTask):

    def __init__(self, task_request):
        super(TakeoffTask, self).__init__()

        self._transition = None
        self._canceled = False
        self._above_min_man_height = False

        try:
            self._TAKEOFF_VELOCITY = rospy.get_param('~takeoff_velocity')
            self._TAKEOFF_ACCELERATION = rospy.get_param('~takeoff_acceleration')
            self._TAKEOFF_COMPLETE_HEIGHT = rospy.get_param('~takeoff_complete_height')
            self._DELAY_BEFORE_TAKEOFF = rospy.get_param('~delay_before_takeoff')
            self._TRANSFORM_TIMEOUT = rospy.get_param('~transform_timeout')
            self._MIN_MANEUVER_HEIGHT = rospy.get_param('~min_maneuver_height')
            self._TAKEOFF_COMPLETE_HEIGHT_TOLERANCE = rospy.get_param('~takeoff_complete_height_tolerance')
            self._TAKEOFF_STABILIZE_DELAY = rospy.get_param('~takeoff_stabilize_delay')
        except KeyError as e:
            rospy.logerr('Could not lookup a parameter for takeoff task')
            raise

        self._state = TakeoffTaskState.init
        self._time_of_ascension = None
        self._time_of_stabilize = None

    def takeoff_callback(self, status, result):
        if status == GoalStatus.SUCCEEDED:
            # Takeoff request succeeded, transition state
            self._state = TakeoffTaskState.ascend
            self._time_of_ascension = rospy.Time.now()
        else:
            # Takeoff request failed
            rospy.logerr('Takeoff task failed during call to low level motion')
            self._state = TakeoffTaskState.failed

    def get_desired_command(self):
        if self._canceled:
            return (TaskCanceled(),)

        # Transition from init to takeoff phase
        if self._state == TakeoffTaskState.init:
            self._state = TakeoffTaskState.takeoff
            return (TaskRunning(),
                    GroundInteractionCommand(
                        'takeoff',
                        self.takeoff_callback),
                    ResetLinearProfileCommand(
                        start_position_z = 0.0,
                        start_velocity_x = 0.0,
                        start_velocity_y = 0.0,
                        start_velocity_z = 0.0))

        # Enter the takeoff phase
        if self._state == TakeoffTaskState.takeoff:
            return (TaskRunning(),)

        if (self._state == TakeoffTaskState.ascend):
            try:
                transStamped = self.topic_buffer.get_tf_buffer().lookup_transform(
                                    'map',
                                    'base_footprint',
                                    rospy.Time(0),
                                    rospy.Duration(self._TRANSFORM_TIMEOUT))
            except (tf2_ros.LookupException,
                    tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException) as ex:
                msg = 'Exception when looking up transform during takeoff'
                rospy.logerr('Takeofftask: {}'.format(msg))
                rospy.logerr(ex.message)
                return (TaskAborted(msg=msg),)

            # Check if we reached the target distance from height
            if(transStamped.transform.translation.z
               + self._TAKEOFF_COMPLETE_HEIGHT_TOLERANCE
               >= self._TAKEOFF_COMPLETE_HEIGHT):
                self._state = TakeoffTaskState.stabilize
                self._time_of_stabilize = rospy.Time.now()
            else:
                velocity = TwistStamped()
                velocity.header.frame_id = 'level_quad'
                velocity.header.stamp = rospy.Time.now()
                velocity.twist.linear.z = min(self._TAKEOFF_VELOCITY,
                    (rospy.Time.now() - self._time_of_ascension).to_sec()
                    * self._TAKEOFF_ACCELERATION)
                return (TaskRunning(), VelocityCommand(velocity))

        if self._state == TakeoffTaskState.stabilize:
            if (rospy.Time.now() - self._time_of_stabilize
                < rospy.Duration(self._TAKEOFF_STABILIZE_DELAY)):
                velocity = TwistStamped()
                velocity.header.frame_id = 'level_quad'
                velocity.header.stamp = rospy.Time.now()
                velocity.twist.linear.z = 0
                return (TaskRunning(), VelocityCommand(velocity, start_position_z=self._TAKEOFF_COMPLETE_HEIGHT))
            else:
                self._state = TakeoffTaskState.done

        if self._state == TakeoffTaskState.done:
            return (TaskDone(), NopCommand())

        if self._state == TakeoffTaskState.failed:
            return (TaskFailed(msg='Take off task experienced failure'),)

        # Impossible state reached
        return (TaskAborted(msg='Impossible state in takeoff task reached'),)

    def cancel(self):
        rospy.loginfo('TakeoffTask cancellation attempted')
        if self._above_min_man_height:
            rospy.loginfo('TakeoffTask cancellation accepted')
            self._canceled = True
            return True
        else: 
            rospy.loginfo('TakeoffTask cancellation rejected')
            return False

    def set_incoming_transition(self, transition):
        self._transition = transition

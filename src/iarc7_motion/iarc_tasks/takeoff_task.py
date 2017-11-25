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
                                      ConfigurePassthroughMode,
                                      AngleThrottleCommand)

from transition_data import TransitionData

class TakeoffTaskState:
    init = 0
    takeoff = 1
    ascend = 2
    done = 3
    failed = 4

class TakeoffTask(AbstractTask):

    def __init__(self, task_request):
        self._transition = None
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)
        self._canceled = False;

        try:
            self._TAKEOFF_VELOCITY = rospy.get_param('~takeoff_velocity')
            self._TAKEOFF_COMPLETE_HEIGHT = rospy.get_param('~takeoff_complete_height')
            self._DELAY_BEFORE_TAKEOFF = rospy.get_param('~delay_before_takeoff')
            self._TRANSFORM_TIMEOUT = rospy.get_param('~transform_timeout')
        except KeyError as e:
            rospy.logerr('Could not lookup a parameter for takeoff task')
            raise

        self._fc_status = None
        self._fc_status_sub = rospy.Subscriber('fc_status', FlightControllerStatus, self._receive_fc_status)
        self._state = TakeoffTaskState.init
        self._arm_request_success = False

    def _receive_fc_status(self, data):
        self._fc_status = data

    def arm_callback(self, data):
        self._arm_request_success = data.success

    def takeoff_callback(self, status, result):
        if status == GoalStatus.SUCCEEDED:
            # Takeoff request succeeded, transition state
            self._state = TakeoffTaskState.ascend
        else:
            # Takeoff request failed
            rospy.logerr('Takeoff task failed during call to low level motion')
            self._state = TakeoffTaskState.failed

    def get_desired_command(self):
        if self._canceled:
            return (TaskCanceled(),)

        # Transition from init to takeoff phase
        elif self._state == TakeoffTaskState.init:
            self._state=TakeoffTaskState.takeoff
            return (TaskRunning(), GroundInteractionCommand(
                                       'takeoff',
                                       self.takeoff_callback))

        # Enter the takeoff phase
        elif self._state == TakeoffTaskState.takeoff:
            return (TaskRunning(),)

        elif (self._state == TakeoffTaskState.ascend):
            
            try:
                transStamped = self._tf_buffer.lookup_transform(
                                    'map',
                                    'base_footprint',
                                    rospy.Time.now(),
                                    rospy.Duration(self._TRANSFORM_TIMEOUT))
            except (tf2_ros.LookupException,
                    tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException) as ex:
                msg = 'Exception when looking up transform during takeoff'
                rospy.logerr('Takeofftask: {}'.format(msg))
                rospy.logerr(ex.message)
                return (TaskAborted(msg=msg),)

            # Check if we reached the target height
            if(transStamped.transform.translation.z > self._TAKEOFF_COMPLETE_HEIGHT):
                self._state = TakeoffTaskState.done
                return (TaskDone(), NopCommand())
        
            else:
                velocity = TwistStamped()
                velocity.header.frame_id = 'level_quad'
                velocity.header.stamp = rospy.Time.now()
                velocity.twist.linear.z = self._TAKEOFF_VELOCITY
                return (TaskRunning(), VelocityCommand(velocity))

        elif self._state == TakeoffTaskState.done:
            return (TaskDone(), NopCommand())

        elif self._state == TakeoffTaskState.failed:
            return (TaskFailed(msg='Take off task experienced failure'),)

        # Impossible state reached
        else:
            return (TaskAborted(msg='Impossible state in takeoff task reached'),)

    def cancel(self):
        rospy.loginfo('TakeoffTask canceled')
        self._canceled = True

    def send_transition(self, transition):
        self._transition = transition

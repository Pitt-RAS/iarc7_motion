#!/usr/bin/env python
import tf2_ros
import math
import rospy

from actionlib_msgs.msg import GoalStatus
from iarc7_msgs.msg import FlightControllerStatus
from geometry_msgs.msg import TwistStamped

from .abstract_task import AbstractTask
from iarc_tasks.task_states import (TaskRunning,
                                    TaskDone,
                                    TaskCanceled,
                                    TaskAborted,
                                    TaskFailed)
from iarc_tasks.task_commands import (NopCommand,
                                      GroundInteractionCommand,
                                      ResetLinearProfileCommand,
                                      VelocityCommand)

class LandTaskState(object):
    init = 0
    land = 1
    done = 2
    failed = 3
    cancelling = 4
    recovering = 5
    cancelled = 6

class LandTask(AbstractTask):
    
    def __init__(self, task_request):
        super(LandTask, self).__init__()

        try:
            self._RECOVERY_HEIGHT = rospy.get_param('~recovery_height')
            self._RECOVERY_VELOCITY = rospy.get_param('~recovery_velocity')
            self._RECOVERY_ACCELERATION = rospy.get_param('~recovery_acceleration')
        except KeyError as e:
            rospy.logerr('Could not lookup a recovery parameter for takeoff task')
            raise

        self._transition = None
        self._state = LandTaskState.init

    def land_callback(self, status, result):
        if status == GoalStatus.SUCCEEDED:
            # cancelling succeeded, so reset linear profile command
            if self._state == LandTaskState.cancelling:
                rospy.loginfo('cancelling has succeeded, time to switch to cancelled state')
                # task succeeded, set state to recovering
                self._state = LandTaskState.recovering
                return
            # task succeeded, set state to done
            self._state = LandTaskState.done
        else:
            # Land request failed
            self._state = LandTaskState.failed

    def get_desired_command(self):

        if self._state == LandTaskState.init:
            # Change state to land
            self._state = LandTaskState.land
            return (TaskRunning(), GroundInteractionCommand(
                       'land',
                       self.land_callback))

        # Enter the landing phase
        if self._state == LandTaskState.land:
            return (TaskRunning(), NopCommand())

        # Change state to done
        if self._state == LandTaskState.done:
            return (TaskDone(), NopCommand())

        # Change state to failed
        if self._state == LandTaskState.failed:
            rospy.logerr('Low level motion failed landing sequence')
            return (TaskFailed(msg='Low level motion failed landing sequence'), NopCommand())

        # Cancel the landing
        if self._state == LandTaskState.cancelling:
            return (TaskRunning(), GroundInteractionCommand(
                       'cancel_land',
                       self.land_callback))

        # Tell linear profile  to get to a recovery height
        if self._state == LandTaskState.recovering:
            rospy.loginfo('LandTask is in recovering state')
            odometry = self.topic_buffer.get_odometry_message()
            # time to deccelerate to recover height
            if odometry.pose.pose.position.z > self._RECOVERY_HEIGHT:
                self._state = LandTaskState.cancelled
                return(TaskRunning(),
                        VelocityCommand(acceleration = self._RECOVERY_ACCELERATION))
            # time to accelerate up
            else:
                velocity = TwistStamped()
                velocity.header.stamp = rospy.Time.now()
                velocity.twist.linear.z = self._RECOVERY_VELOCITY
                return(TaskRunning(),
                        VelocityCommand(
                            target_twist = velocity,
                            start_position_z = odometry.pose.pose.position.z,
                            start_velocity_x = 0.0,
                            start_velocity_y = 0.0,
                            start_velocity_z = odometry.twist.twist.linear.z,
                            acceleration = self._RECOVERY_ACCELERATION))

        if self._state == LandTaskState.cancelled:
            rospy.loginfo('LandTask is cancelled and ready for next task')
            return(TaskCanceled(),)

        # Impossible state reached
        return (TaskAborted(msg='Impossible state in takeoff task reached'))

    def set_incoming_transition(self, transition):
        self._transition = transition

    def cancel(self):
        rospy.loginfo('LandTask cancellation requested')

        if self._state == LandTaskState.done: 
            rospy.loginfo('LandTask cancellation accepted')
            return True
        elif self._state == LandTaskState.land:
            rospy.loginfo('LandTask sending cancellation to LLM')
            self._state = LandTaskState.cancelling
            return False
        elif self._state == LandTaskState.cancelling:
            rospy.loginfo('LandTask in process of cancelling')
            return False
        elif self._state == LandTaskState.recovering:
            rospy.loginfo('LandTask is waiting to recover height')
            return False
        elif self._state == LandTaskState.cancelled:
            rospy.loginfo('Land Task has cancelled')
            return True 
        else: 
            rospy.loginfo('LandTask cancellation rejected')
            return False

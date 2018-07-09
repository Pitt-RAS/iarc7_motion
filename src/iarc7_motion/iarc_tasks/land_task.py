#!/usr/bin/env python
import tf2_ros
import math
import rospy

from actionlib_msgs.msg import GoalStatus
from iarc7_msgs.msg import FlightControllerStatus

from .abstract_task import AbstractTask
from iarc_tasks.task_states import (TaskRunning,
                                    TaskDone,
                                    TaskCanceled,
                                    TaskAborted,
                                    TaskFailed)
from iarc_tasks.task_commands import (NopCommand,
                                      GroundInteractionCommand,
                                      ResetLinearProfileCommand)

class LandTaskState(object):
    init = 0
    land = 1
    done = 2
    failed = 3
    cancelling = 4
    cancelled = 5

class LandTask(AbstractTask):
    
    def __init__(self, task_request):
        super(LandTask, self).__init__()

        try:
            self._TRANSFORM_TIMEOUT = rospy.get_param('~transform_timeout')
        except KeyError as e:
            rospy.logerr('Could not lookup a parameter for takeoff task')
            raise

        self._transition = None
        self._state = LandTaskState.init

    def land_callback(self, status, result):
        if status == GoalStatus.SUCCEEDED:
            # cancelling succeeded, so reset linear profile command
            if self._state == LandTaskState.cancelling:
                rospy.loginfo('cancelling has succeeded, time to ResetLinearProfileCommand')
                try:
                    transStamped = self.topic_buffer.get_tf_buffer().lookup_transform(
                                        'map',
                                        'base_footprint',
                                        rospy.Time(0),
                                        rospy.Duration(self._TRANSFORM_TIMEOUT))
                except (tf2_ros.LookupException,
                        tf2_ros.ConnectivityException,
                        tf2_ros.ExtrapolationException) as ex:
                    msg = 'Exception when looking up transform during land cancelling'
                    rospy.logerr('LandTask: {}'.format(msg))
                    rospy.logerr(ex.message)
                    return (TaskAborted(msg=msg),)  
                # task succeeded, set state to cancelled
                self._state = LandTaskState.cancelled
                return(TaskCanceled(),
                        ResetLinearProfileCommand(
                            start_position_z = transStamped.transform.translation.z,
                            start_velocity_x = 0.0,
                            start_velocity_y = 0.0,
                            start_velocity_z = 0.0))
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
            return (TaskRunning(), NopCommand());

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
        elif self._state == LandTaskState.cancelled:
            rospy.loginfo('LandTask has finished cancelling')
            return True
        else: 
            rospy.loginfo('LandTask cancellation rejected')
            return False

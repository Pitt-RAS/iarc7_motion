#!/usr/bin/env python

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
                                      GroundInteractionCommand)

class LandTaskState(object):
    init = 0
    land = 1
    done = 2
    failed = 3

class LandTask(object, AbstractTask):
    
    def __init__(self, task_request):
        self._transition = None
        self._canceled = False

        self._fc_status = None
        self._fc_status_sub = rospy.Subscriber('fc_status', FlightControllerStatus, self._receive_fc_status)
        self._state = LandTaskState.init

    def _receive_fc_status(self, data):
        self._fc_status = data

    def land_callback(self, status, result):
        if status == GoalStatus.SUCCEEDED:
            # Land request succeeded, transition state
            self._state = LandTaskState.done
        else:
            # Land request failed
            self._state = LandTaskState.failed

    def get_desired_command(self):
        if self._canceled:
            return (TaskCanceled(),)

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

        # Impossible state reached
        return (TaskAborted(msg='Impossible state in takeoff task reached'))

    def set_incoming_transition(self, transition):
        self._transition = transition

    def cancel(self):
        rospy.loginfo('LandTask cancellation requested')

        if self._state == LandTaskState.done: 
            rospy.loginfo('LandTask cancellation accepted')
            self._canceled = True
            return True
        else: 
            rospy.loginfo('LandTask cancellation rejected')
            return False

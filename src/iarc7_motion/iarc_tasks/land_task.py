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
from iarc_tasks.task_commands import (VelocityCommand,
                                      ArmCommand,
                                      NopCommand,
                                      GroundInteractionCommand)

class LandTaskState:
    init = 0
    land = 1
    disarm = 2
    failed = 3

class LandTask(AbstractTask):

    def __init__(self, actionvalues_dict):
        self._canceled = False;

        self._fc_status = None
        self._fc_status_sub = rospy.Subscriber('fc_status', FlightControllerStatus, self._receive_fc_status)
        self._state = LandTaskState.init
        self._disarm_request_success = False

    def _receive_fc_status(self, data):
        self._fc_status = data

    def disarm_callback(self, data):
        self._disarm_request_success = data.success

    def land_callback(self, status, result):
        if status == GoalStatus.SUCCEEDED:
            # Takeoff request succeeded, transition state
            self._state = LandTaskState.disarm
        else:
            # Takeoff request failed
            rospy.logerr('Takeoff task failed during call to low level motion')
            self._state = LandTaskState.failed

    def get_desired_command(self):
        if self._canceled:
            return (TaskCanceled(),)

        if self._state == LandTaskState.init:
            # Check if we have a fc status
            if self._fc_status is None:
                return (TaskRunning(), NopCommand())
            # Check that auto pilot is enabled
            if not self._fc_status.auto_pilot:
                return (TaskFailed(msg='flight controller not allowing auto pilot'),)
            # Check that the FC is not already armed
            if not self._fc_status.armed:
                return (TaskFailed(msg='flight controller is not armed, how are we in the air??'),)
            # All is good change state to land
            self._state = LandTaskState.land
            return (TaskRunning(), GroundInteractionCommand(
                       'land',
                       self.land_callback))

        # Enter the takeoff phase
        if self._state == LandTaskState.land:
            return (TaskRunning(), NopCommand());

        # Enter the disarming request stage
        if self._state == LandTaskState.disarm:
            # Check if disarm succeeded
            if self._disarm_request_success:
                return (TaskDone(), NopCommand())
            else:
                return (TaskRunning(), ArmCommand(False, self.disarm_callback))

        # Enter the disarming request stage
        if self._state == LandTaskState.failed:
            # Check if disarm succeeded
            if self._disarm_request_success:
                return (TaskFailed(), NopCommand())
            else:
                rospy.logerr('Low level motion action failed, disarming then exiting')
                return (TaskRunning(), ArmCommand(False, self.disarm_callback))

        # Impossible state reached
        return (TaskAborted(msg='Impossible state in takeoff task reached'))

    def cancel(self):
        rospy.loginfo('TakeoffTask canceled')
        self._canceled = True

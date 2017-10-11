#!/usr/bin/env python

class IntermediaryState(object):

    def __init__(self, drone_odometry = None, roomba_odometry = None, 
            last_command = None, last_task_ending_state = None, arm_status = None):
        """
        Intermediary State

        Args:
            drone_odometry: odometry (position, velocity, etc.) of drone
            roomba_odometry: odometry (position, velocity, etc.) of
                all roombas in sight of drone
            last_command: last task command 
            last_task_ending_state: last task ending state
            arm_status: current arm status of drone
        """
        self.drone_odometry = drone_odometry
        self.roomba_odometry = roomba_odometry
        self.last_command = last_command
        self.last_task_ending_state = last_task_ending_state
        self.arm_status = arm_status

    @property
    def drone_odometry(self):
        return self._drone_odometry

    @drone_odometry.setter
    def drone_odometry(self, drone_odometry):
        self._drone_odometry = drone_odometry

    @property
    def roomba_odometry(self):
        return self._roomba_odometry

    @roomba_odometry.setter
    def roomba_odometry(self, roomba_odometry):
        self._roomba_odometry = roomba_odometry

    @property
    def last_command(self):
        return self._last_command

    @last_command.setter
    def last_command(self, last_command):
        self._last_command = last_command

    @property
    def last_task_ending_state(self):
        return self._last_task_ending_state

    @last_task_ending_state.setter
    def last_task_ending_state(self, last_task_ending_state):
        self._last_task_ending_state = last_task_ending_state

    @property
    def arm_status(self):
        return self._arm_status

    @arm_status.setter
    def arm_status(self, arm_status):
        self._arm_status = arm_status

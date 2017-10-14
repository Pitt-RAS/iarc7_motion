#!/usr/bin/env python

class IntermediaryState(object):

    def __init__(self, drone_odometry = None, roombas = None, 
            timeout_sent = None, last_task_ending_state = None, arm_status = None, last_twist = None):
        """
        Intermediary State

        Args:
            drone_odometry: odometry (position, velocity, etc.) of drone
            roombas: odometry (position, velocity, etc.) of
                all roombas in sight of drone
            timeout_sent: whether or not timeout was sent
            last_task_ending_state: last task ending state
            arm_status: current arm status of drone
            last_twist: last twist (velocity request) sent to LLM
        """
        self.drone_odometry = drone_odometry
        self.roombas = roombas
        self.timeout_sent = timeout_sent
        self.last_task_ending_state = last_task_ending_state
        self.arm_status = arm_status
        self.last_twist = last_twist

    @property
    def drone_odometry(self):
        return self._drone_odometry

    @drone_odometry.setter
    def drone_odometry(self, drone_odometry):
        self._drone_odometry = drone_odometry

    @property
    def roombas(self):
        return self._roombas

    @roombas.setter
    def roombas(self, roombas):
        self._roombas = roombas

    @property
    def timeout_sent(self):
        return self._timeout_sent

    @timeout_sent.setter
    def timeout_sent(self, timeout_sent):
        self._timeout_sent = timeout_sent

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

    @property
    def last_twist(self):
        return self._last_twist

    @last_twist.setter
    def last_twist(self, twist):
        self._last_twist = twist
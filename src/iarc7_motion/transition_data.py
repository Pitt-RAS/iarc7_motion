#!/usr/bin/env python

class TransitionData(object):

    def __init__(self, drone_odometry=None,
                       roombas=None,
                       obstacles=None,
                       last_task_ending_state=None,
                       arm_status=None,
                       last_twist=None):
        """
        Transition Data

        Args:
            drone_odometry: odometry (position, velocity, etc.) of drone
            roombas: odometry (position, velocity, etc.) of
                all roombas in sight of drone
            obstacles: odometry (position, velocity, etc.) of
                all obstacles in sight of drone
            last_task_ending_state: last task ending state
            arm_status: current arm status of drone
            last_twist: last twist (velocity request) sent to LLM
        """
        self.drone_odometry = drone_odometry
        self.roombas = roombas
        self.obstacles = obstacles
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
    def obstacles(self):
        return self._obstacles

    @obstacles.setter
    def obstacles(self, obstacles):
        self._obstacles = obstacles

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

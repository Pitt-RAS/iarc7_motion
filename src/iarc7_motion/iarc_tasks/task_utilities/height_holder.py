#!/usr/bin/env python

"""
HeightHolder: provides a simple means for a task to get a z-velocity
response that maintains a certain height above the ground

"""

import rospy
import threading

class HeightHolder(object):
    def __init__(self, desired_height = None):
        self._lock = threading.RLock()
        self._delta_z = 0
        self._DESIRED_HEIGHT = desired_height
        try:
            self._MIN_MANEUVER_HEIGHT = rospy.get_param('~min_maneuver_height')
            self._MAX_Z_ERROR = rospy.get_param('~max_z_error')
            self._K_Z = rospy.get_param('~p_term_height_hold_z')
        except KeyError as e:
            rospy.logerr('Could not lookup a parameter for track roomba task')
            raise

        if self._DESIRED_HEIGHT < self._MIN_MANEUVER_HEIGHT and self._DESIRED_HEIGHT is not None:
            raise ValueError('Desired height was below the minimum maneuver height')

    # uses a p-controller to return a velocity to maintain a height
    def get_height_hold_response(self, height):
        with self._lock:
            if self._DESIRED_HEIGHT is None:
                raise ValueError('No height to hold')
            self._delta_z = self._DESIRED_HEIGHT - height
            return self._K_Z * self._delta_z

    def set_height(self, desired_height):
         if desired_height < self._MIN_MANEUVER_HEIGHT:
            raise ValueError('Requested height is too low')

         self._DESIRED_HEIGHT = desired_height

    def check_z_error(self, current_height):
        return (abs(self._delta_z) < self._MAX_Z_ERROR)

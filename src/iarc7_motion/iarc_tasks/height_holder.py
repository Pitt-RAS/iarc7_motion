#!/usr/bin/env python

# A helper class for a task that will spit out a velocity that's appropriate
# to reach and hold a desired position in the z dimension
# Uses a simple p-controller

import math
import rospy
import threading

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry

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
                if (height is None):
                    raise ValueError('There is no height to hold at')
                else:
                    self._DESIRED_HEIGHT = height

            self._delta_z = self._DESIRED_HEIGHT - height
            return self._K_Z * self._delta_z

    def check_z_error(self, current_height):
        return (abs(self._delta_z) < self._MAX_Z_ERROR)

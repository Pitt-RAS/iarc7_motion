#!/usr/bin/env python

# A helper class for a task that will determine if 
# the drone is above min maneuver height 
# and if the z error is too high 

import math
import rospy
import threading

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry

class HeightChecker():
    def __init__(self):
        self._lock = threading.RLock()
        try:
            self._MIN_MANEUVER_HEIGHT = rospy.get_param('~min_maneuver_height')
            self._MAX_Z_ERROR = rospy.get_param('~max_z_error')
        except KeyError as e:
            rospy.logerr('Could not lookup a parameter for track roomba task')
            raise
            
    def above_min_maneuver_height(self, current_height):
        with self._lock:
            if (current_height < self._MIN_MANEUVER_HEIGHT):
                return False
            else:
                return True

    def check_z_error(self, current_height, desired_height):
        with self._lock:
            z_error = abs(current_height-desired_height)
            if (z_error > self._MAX_Z_ERROR):
                return False
            else:
                return True

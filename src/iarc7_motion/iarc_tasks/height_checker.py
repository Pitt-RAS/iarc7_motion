#!/usr/bin/env python

# A helper class for a task that will spit out a velocity that's appropriate
# to reach and hold a desired position in the z dimension
# Uses a simple p-controller

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
        except KeyError as e:
            rospy.logerr('Could not lookup a parameter for track roomba task')
            raise

    # uses a p-controller to return a velocity to maintain a height
    # that is set as the param _track_roomba_height
    def above_min_maneuver_height(self, height):
        with self._lock:
            if height  < self._MIN_MANEUVER_HEIGHT:
                return False
            else:
                return True

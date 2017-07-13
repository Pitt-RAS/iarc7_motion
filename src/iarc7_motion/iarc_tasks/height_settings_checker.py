#!/usr/bin/env python

# A helper class for a task that will determine if 
# the drone is above min maneuver height 
# and if the z error is too high 

import math
import rospy

class HeightSettingsChecker():
    def __init__(self):
        try:
            self._MIN_MANEUVER_HEIGHT = rospy.get_param('~min_maneuver_height')
        except KeyError as e:
            rospy.logerr('Could not lookup a parameter Height Settings Checker')
            raise

    def above_min_maneuver_height(self, current_height):
        if (current_height < self._MIN_MANEUVER_HEIGHT):
            return False
        else:
            return True

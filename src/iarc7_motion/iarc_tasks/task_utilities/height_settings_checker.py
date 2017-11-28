#!/usr/bin/env python

"""
HeightSettingsChecker: checks that passed in settings are 
    above minimum manuever height.

"""

import rospy

class HeightSettingsChecker(object):
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

#!/usr/bin/env python

"""
Task Utilities- series of task helpers and utilities

Types: 
    AccelerationLimiter: provides a utility to limit acceleration vectors
    HeightHolder: provides a simple means for a task to get a z-velocity 
        response that maintains a certain height above the ground
    HeightSettingsChecker: checks that passed in settings are
        above minimum manuever height. 

"""

import math
import rospy
import threading

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry

# A helper class for a task that will limit acceleration vectors
class AccelerationLimiter(object):
    def __init__(self):
        try:
            self._MAX_3D_TRANSLATION_ACCELERATION = rospy.get_param('~max_translation_acceleration')
            update_rate = rospy.get_param('~update_rate', False)
        except KeyError as e:
            rospy.logerr('Could not lookup a parameter for Acceleration Limiter')
            raise
        self._update_period = 1.0/update_rate

    def limit_acceleration(self, current_velocities, desired_velocities):
        return_velocities = []

        current_x = current_velocities[0]
        current_y = current_velocities[1]
        current_z = current_velocities[2]

        desired_x = desired_velocities[0]
        desired_y = desired_velocities[1]
        desired_z = desired_velocities[2]

        accel_x = (desired_x - current_x)/self._update_period
        accel_y = (desired_y - current_y)/self._update_period
        accel_z = (desired_z - current_z)/self._update_period

        accel_overall = math.sqrt(accel_x**2 + accel_y**2 + accel_z**2)

        if accel_overall > self._MAX_3D_TRANSLATION_ACCELERATION:
            return_velocities.append(current_x + ((self._MAX_3D_TRANSLATION_ACCELERATION * 
                self._update_period) * (accel_x/accel_overall)))

            return_velocities.append(current_y + ((self._MAX_3D_TRANSLATION_ACCELERATION * 
                self._update_period) * (accel_y/accel_overall)))

            return_velocities.append(current_z + ((self._MAX_3D_TRANSLATION_ACCELERATION * 
                self._update_period) * (accel_z/accel_overall)))
        else:
            return_velocities.append(desired_x)
            return_velocities.append(desired_y)
            return_velocities.append(desired_z)
        return return_velocities


# A helper class for a task that will spit out a velocity that's appropriate
# to reach and hold a desired position in the z dimension
# Uses a simple p-controller

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


# A helper class for a task that will say whether or not a setting is
# above minimum manuever height
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

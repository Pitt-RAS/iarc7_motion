#!/usr/bin/env python

# A helper class for a task that will limit acceleration vectors

import math
import rospy

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

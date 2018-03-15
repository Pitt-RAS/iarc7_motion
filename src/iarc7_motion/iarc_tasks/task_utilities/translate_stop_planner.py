#!/usr/bin/env python

# A helper class for a task that will spit out a velocity that's appropriate
# to reach and hold a desired position in three dimensions

import math
import threading

import rospy

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry

class TranslateStopPlanner():
    def __init__(self, x=None, y=None, z=None):
        update_rate = rospy.get_param('~update_rate', False)
        self._update_period = 1.0/update_rate
        self._lock = threading.RLock()
        self._odometry = None
        self._hold_x = x
        self._hold_y = y
        self._hold_z = z
        self._control_lag = rospy.get_param('~control_lag', 0.0)
        self._max_acceleration = rospy.get_param('~max_translation_acceleration', 0.0)
        self._desired_acceleration = rospy.get_param('~desired_translation_acceleration', 0.0)
        self._max_speed = rospy.get_param('~max_translation_speed', 0.0)
        self._position_tolerance = rospy.get_param('~translation_position_hold_tolerance', 0.0)
        self._current_velocity_sub = rospy.Subscriber('/odometry/filtered',
                                              Odometry,
                                              self._current_velocity_callback)
        self._last_vel_x = None
        self._last_vel_y = None
        self._last_vel_z = None
        self._done = False
        self._last_actual_twist = None
        self._last_target_acceleration = 0.0

    def get_xyz_hold_response(self):
        with self._lock:
            if self._odometry is not None:
                delta_x = self._hold_x - self._odometry.pose.pose.position.x
                delta_y = self._hold_y - self._odometry.pose.pose.position.y
                delta_z = self._hold_z - self._odometry.pose.pose.position.z
                response = self._get_next_acceleration_target_trapezoidal(delta_x,
                                                                          delta_y,
                                                                          delta_z)
            else:
                rospy.logerr('get_xy_hold_response called before odometry published')
                response = TwistStamped()
                response.header.stamp = rospy.Time.now()
                response.header.frame_id = 'level_quad'
            return response

    def reinitialize_translation_stop_planner(self, x=None, y=None, z=None):
        with self._lock:
            self._hold_x = x
            self._hold_y = y
            self._hold_z = z


    def _current_velocity_callback(self, odometry):
        with self._lock:
            if (odometry.header.frame_id == 'map'
                and odometry.child_frame_id == 'level_quad'):
                self._odometry = odometry
                if self._hold_x is None:
                    self._hold_x = odometry.pose.pose.position.x
                if self._hold_y is None:
                    self._hold_y = odometry.pose.pose.position.y
                if self._hold_z is None:
                    self._hold_z = odometry.pose.pose.position.z
                if self._last_vel_x is None:
                    self._last_vel_x = odometry.twist.twist.linear.x
                if self._last_vel_y is None:
                    self._last_vel_y = odometry.twist.twist.linear.y
                if self._last_vel_z is None:
                    self._last_vel_z = odometry.twist.twist.linear.z
                self._last_actual_twist = odometry.twist.twist
            else:
                rospy.logwarn('Received odometry message with incorrect frame and child frames')

    def _calculate_triangle_acceleration(self,
                                         x, y, z,
                                         distance,
                                         integrated_speed_towards_target):

        # Compute a dot product to find our measured speed towards target
        measured_speed_towards_target = ((self._last_actual_twist.linear.x
                                        * x
                                        + self._last_actual_twist.linear.y
                                        * y
                                        + self._last_actual_twist.linear.z
                                        * z)
                                        / distance)

        # Estimate how far we will travel before an acceleration is applied
        # Uses v*t - 0.5*a*t^2
        translation_till_accel_applied = ((measured_speed_towards_target
                                         * self._control_lag)
                                         - (0.5
                                         * self._last_target_acceleration
                                         * (self._control_lag ** 2)))

        # If there is still time to apply an acceleration
        if distance > translation_till_accel_applied:
            distance_when_accel_applied = (distance -
                                           translation_till_accel_applied)
            required_deceleration = ((integrated_speed_towards_target**2)
                                    /(2*distance_when_accel_applied))
            # We want to maintain a certain acceleration and deceleration
            # profile. If we cross this point it's time to decelerate usually
            # otherwise just keep accelerating
            if required_deceleration < self._desired_acceleration:
                # If these two things are true we've been decelerating too fast
                return self._desired_acceleration
            else:
                return -required_deceleration
        else:
            rospy.logdebug('the acceleration better have worked')
            # We better hope the other accelerations worked. Velocity
            # will be set to zero soon
            return 0.0


    def _get_next_acceleration_target_trapezoidal(self, x, y, z):
        # calculate straight line distance
        distance = math.sqrt(x**2 + y**2 + z**2)

        # Compute a dot product to find our requested speed towards a target
        integrated_speed_towards_target = ((self._last_vel_x
                                          * x
                                          + self._last_vel_y
                                          * y
                                          + self._last_vel_z
                                          * z)
                                          / distance)

        # We will cap the speed from this acceleration if necessary to
        # create a trapezoidal velocity path
        target_acceleration = self._calculate_triangle_acceleration(
                                   x, y, z,
                                   distance,
                                   integrated_speed_towards_target)

        # Calculate the desired speed from the acceleration
        # This is a future requested velocity
        desired_speed_towards_target = (integrated_speed_towards_target
                                       + (target_acceleration
                                       * self._update_period))

        # Calculate the target vx vector
        target_vx = (x/distance) * desired_speed_towards_target
        target_vy = (y/distance) * desired_speed_towards_target
        target_vz = (z/distance) * desired_speed_towards_target

        # Calculate the error velocity by subtracting our integrated velocity
        # from the target velocity
        error_vx = target_vx - self._last_vel_x
        error_vy = target_vy - self._last_vel_y
        error_vz = target_vz - self._last_vel_z
        error_v = math.sqrt(error_vx**2 + error_vy**2 + error_vz**2)

        # Calculate the overall acceleration
        total_acceleration = error_v / self._update_period

        # Cap the max acceleration by scaling the vector as necessary
        if total_acceleration > self._max_acceleration:
            error_vx = error_vx * ((self._max_acceleration
                                  * self._update_period) / error_v)
            error_vy = error_vy * ((self._max_acceleration
                                  * self._update_period) / error_v)
            error_vz = error_vz * ((self._max_acceleration
                                  * self._update_period) / error_v)
            rospy.logwarn('Hit max acceleration limits in position holder!')

        # Fill out the twist
        target_twist = TwistStamped()
        target_twist.header.stamp = rospy.Time.now()
        target_twist.header.frame_id = 'level_quad'

        if distance > self._position_tolerance:
            target_twist.twist.linear.x = (self._last_vel_x
                                          + error_vx)
            target_twist.twist.linear.y = (self._last_vel_y
                                          + error_vy)
            target_twist.twist.linear.z = (self._last_vel_z
                                          + error_vz)
        else:
            # We are assuming that the accelerations worked at this point.
            # Maybe in the future use an interpolator to bring
            # the integrated velocity to zero
            target_twist.twist.linear.x = 0.0
            target_twist.twist.linear.y = 0.0
            target_twist.twist.linear.z = 0.0
            self._done = True
            rospy.logdebug('tolerance hit')

        # Cap the final speed request
        requested_speed = math.sqrt(target_twist.twist.linear.x**2
                                    + target_twist.twist.linear.y**2
                                    + target_twist.twist.linear.z**2)
        if requested_speed > self._max_speed:
            target_twist.twist.linear.x = (target_twist.twist.linear.x
                                          * self._max_speed / requested_speed)
            target_twist.twist.linear.y = (target_twist.twist.linear.y
                                          * self._max_speed / requested_speed)
            target_twist.twist.linear.z = (target_twist.twist.linear.z
                                          * self._max_speed / requested_speed)

        # Save off the state variables
        self._last_vel_x = target_twist.twist.linear.x
        self._last_vel_y = target_twist.twist.linear.y
        self._last_vel_z = target_twist.twist.linear.z
        self._last_target_acceleration = target_acceleration

        return target_twist

    def is_done(self):
        return self._done

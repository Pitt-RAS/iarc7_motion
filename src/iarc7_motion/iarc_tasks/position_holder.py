#!/usr/bin/env python

# A helper class for a task that will spit out a velocity that's appropriate
# to reach and hold a desired position in three dimensions

import math
import threading

import rospy

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry

class PositionHolder():
    def __init__(self, x=None, y=None, z=None):
        update_rate = rospy.get_param('~update_rate', False)
        self._update_period = 1.0/update_rate
        self._lock = threading.RLock()
        self._odometry = None
        self._hold_x = x
        self._hold_y = y
        self._hold_z = z
        self._max_acceleration = rospy.get_param('~max_translation_acceleration', 0.0)
        self._max_speed = rospy.get_param('~max_translation_speed', 0.0)
        self._position_tolerance = rospy.get_param('~translation_position_hold_tolerance', 0.0)
        self._current_velocity_sub = rospy.Subscriber('/odometry/filtered',
                                              Odometry,
                                              self._current_velocity_callback)
        self._last_vel_x = None
        self._last_vel_y = None
        self._last_vel_z = None
        self._last_speed = None

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
                if self._last_speed is None:
                    self._last_speed = (
                        math.sqrt(odometry.twist.twist.linear.x**2
                                  + odometry.twist.twist.linear.y**2
                                  + odometry.twist.twist.linear.z**2))
            else:
                rospy.logwarn('Received odometry message with incorrect frame and child frames')

    def _calculate_trapezoidal_acceleration(self, x, y, z, distance):
        try:
            # Compute a dot product to find our current speed towards a target
            # The z term can be discarded since we aren't concerned with the z velocity
            speed_towards_target = ((self._last_vel_x * x + self._last_vel_y * y
                                    + self._last_vel_z * z) / distance)
        except ZeroDivisionError:
            # Last speed was zero therefore the last speed to the target was 0
            speed_towards_target = 0

        if speed_towards_target > 0:
            # Check if we can decelerate in time
            try:
                required_deceleration = (speed_towards_target**2)/(2*distance)
            except ZeroDivisionError:
                # We are at the target point, no need to accelerate anywhere
                return 0.0

            if required_deceleration < self._max_acceleration:
                if self._last_speed < self._max_speed:
                    return self._max_acceleration
                else:
                    return 0.0
            else:
                return -self._max_acceleration

        else:
            # speed is less than zero or zero
            if self._last_speed < self._max_speed:
                return self._max_acceleration
            else:
                return 0.0

    def _get_next_acceleration_target_trapezoidal(self, x, y, z):
        # calculate straight line distance
        distance = math.sqrt(x**2 + y**2 + z**2)

        target_acceleration = self._calculate_trapezoidal_acceleration(x, y, z,
                                                              distance)
        # current_angle = math.atan2(y, x)
        # current_vertical_angle = math.atan2(z, x)

        target_twist = TwistStamped()
        target_twist.header.stamp = rospy.Time.now()
        target_twist.header.frame_id = 'level_quad' # TODO

        if distance > self._position_tolerance:
            target_twist.twist.linear.x = ((self._last_speed
                                          + (target_acceleration
                                          *  self._update_period))
                                          * (x/distance))
            target_twist.twist.linear.y = ((self._last_speed
                                          + (target_acceleration
                                          * self._update_period))
                                          * (y/distance))
            target_twist.twist.linear.z = ((self._last_speed
                                          + (target_acceleration
                                          * self._update_period))
                                          * (z/distance))
        else:
            target_twist.twist.linear.x = 0.0
            target_twist.twist.linear.y = 0.0
            target_twist.twist.linear.z = 0.0
            rospy.logdebug('tolerance hit')

        self._last_vel_x = target_twist.twist.linear.x
        self._last_vel_y = target_twist.twist.linear.y
        self._last_vel_z = target_twist.twist.linear.z
        self._last_speed = math.sqrt(self._last_vel_x**2 + self._last_vel_y**2
                                     + self._last_vel_z**2)

        # rospy.logdebug('ta %s current angle %s',
        #                 target_acceleration,
        #                 current_angle)
        rospy.logdebug('target acceleration: %s', target_acceleration)
        rospy.logdebug('dx %s dy %s dz %s dvx %s dvy %s dvz %s vx %s vy %s vz %s', x, y, z,
        target_acceleration * self._update_period * -(x/distance),
        target_acceleration * self._update_period * -(y/distance),
        target_acceleration * self._update_period * -(z/distance),
        target_twist.twist.linear.x, target_twist.twist.linear.y,
        target_twist.twist.linear.z)

        # Set a target accleration by setting velocity that's timestamped into the future based on the current velocity
        return target_twist

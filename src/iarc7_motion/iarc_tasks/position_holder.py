#!/usr/bin/env python

# A helper class for a task that will spit out a velocity that's appropriate
# to reach and hold a desired position

import math
import threading

import rospy

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry

class PositionHolder():
    def __init__(self, x=None, y=None):
        update_rate = rospy.get_param('~update_rate', False)
        self._update_period = 1.0/update_rate
        self._lock = threading.RLock()
        self._odometry = None
        self._hold_x = x
        self._hold_y = y
        self._max_acceleration = rospy.get_param('~max_translation_acceleration', 0.0)
        self._max_speed = rospy.get_param('~max_translation_speed', 0.0)
        self._position_tolerance = rospy.get_param('~translation_position_hold_tolerance', 0.0)
        self._current_velocity_sub = rospy.Subscriber('/odometry/filtered',
                                              Odometry,
                                              self._current_velocity_callback)
        self._last_vel_x = None
        self._last_vel_y = None
        self._last_speed = None

    def get_xy_hold_response(self, z_velocity=None):
        with self._lock:
            if self._odometry is not None:
                delta_x = self._hold_x - self._odometry.pose.pose.position.x
                delta_y = self._hold_y - self._odometry.pose.pose.position.y
                response = self._get_next_acceleration_target_trapezoidal(delta_x,
                                                                          delta_y,
                                                                          z_velocity=z_velocity)
            else:
                rospy.logerr('get_xy_hold_response called before odometry published')
                response = TwistStamped()
                response.header.stamp = rospy.Time.now()
                response.header.frame_id = 'level_quad'

            return response

    def _current_velocity_callback(self, odometry):
        with self._lock:
            if odometry.header.frame_id == 'map' and odometry.child_frame_id == 'level_quad':
                self._odometry = odometry
                if self._hold_x is None:
                    self._hold_x = odometry.pose.pose.position.x
                if self._hold_y is None:
                    self._hold_y = odometry.pose.pose.position.y
                if self._last_vel_x is None:
                    self._last_vel_x = odometry.twist.twist.linear.x
                if self._last_vel_y is None:
                    self._last_vel_y = odometry.twist.twist.linear.y
                if self._last_speed is None:
                    self._last_speed = math.sqrt(self._last_vel_x**2 + self._last_vel_y**2)
            else:
                rospy.logwarn('Received odometry message with incorrect frame and child frames')

    def _calculate_trapezoidal_acceleration(self, distance, speed, speed_towards_target):
        
        if speed_towards_target > 0:
            # Check if we can decelerate in time
            try:
                target_acceleration = (speed**2)/(2*distance)
            except ZeroDivisionError:
                # We are at the target point, no need to accelerate anywhere
                return 0.0

            if target_acceleration < self._max_acceleration:
                if speed < self._max_speed:
                    target_acceleration = self._max_acceleration
                else:
                    target_acceleration = 0.0
            else:
                target_acceleration = -self._max_acceleration

            return target_acceleration
        else:
            # speed is less than zero or zero
            if speed < self._max_speed:
                return self._max_acceleration
            else:
                return 0.0

    def _get_next_acceleration_target_trapezoidal(self, x, y, z_velocity=None):
        # calculate current straight line velocity
        last_speed = math.sqrt(self._last_vel_x**2 + self._last_vel_y**2)

        try:
            speed_towards_target = ((self._last_vel_x * x + self._last_vel_y * y)
                                    / last_speed)
        except ZeroDivisionError:
            # Last speed was zero therefore the last speed to the target was 0
            speed_towards_target = 0

        # calculate straight line distance
        distance = math.sqrt(x**2 + y**2)

        target_acceleration = self._calculate_trapezoidal_acceleration(distance, last_speed, speed_towards_target)

        current_angle = math.atan2(y, x)

        target_twist = TwistStamped()
        target_twist.header.stamp = rospy.Time.now()
        target_twist.header.frame_id = 'level_quad'

        if distance > self._position_tolerance:
            target_twist.twist.linear.x = ((last_speed
                                          + (target_acceleration
                                          *  self._update_period))
                                          * math.cos(current_angle))
            target_twist.twist.linear.y = ((last_speed
                                          + (target_acceleration
                                          * self._update_period))
                                          * math.sin(current_angle))
        else:
            target_twist.twist.linear.x = 0.0
            target_twist.twist.linear.y = 0.0
            rospy.logdebug('tolerance hit')

        self._last_vel_x = target_twist.twist.linear.x
        self._last_vel_y = target_twist.twist.linear.y

        rospy.logdebug('ta %s current angle %s', target_acceleration, current_angle)
        rospy.logdebug('dx %s dy %s dvx %s dvy %s vx %s vy %s', x, y,
         target_acceleration * self._update_period * -math.cos(current_angle),
         target_acceleration * self._update_period * -math.sin(current_angle),
         target_twist.twist.linear.x, target_twist.twist.linear.y)

        if z_velocity is not None:
            target_twist.twist.linear.z = z_velocity

        # Set a target accleration by setting velocity that's timestamped into the future based on the current velocity
        return target_twist

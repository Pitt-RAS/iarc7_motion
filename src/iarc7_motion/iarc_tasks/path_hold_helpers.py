#!/usr/bin/env python

# A collection of helper functions that allow tasks to hold desired
# position or path relative to an axis

import threading

import rospy

import math

from geometry_msgs.msg import TwistStamped

from nav_msgs.msg import Odometry

class PositionHolder():
    def __init__(self, x, y):
        update_rate = rospy.get_param('~update_rate', False)
        self._update_period = 1.0/update_rate
        self._lock = threading.RLock()
        self._odometry = None
        self._hold_x = x
        self._hold_y = y
        self._current_velocity_sub = rospy.Subscriber('/odometry/filtered',
                                              Odometry,
                                              self._current_velocity_callback)

    def get_xy_hold_response(self, current_x, current_y, z_velocity=None):
        with self._lock:
            delta_x = self._hold_x - current_x
            delta_y = self._hold_y - current_y
            if self._odometry is not None:
                response = self._get_next_acceleration_target_trapezoidal(delta_x,
                                                                          delta_y,
                                                                          self._odometry.twist,
                                                                          z_velocity=z_velocity)
            else:
                rospy.logerr('get_xy_hold_response called before odometry published')

            return response

    def _current_velocity_callback(self, odometry):
        with self._lock:
            if odometry.header.frame_id == 'map' and odometry.child_frame_id == 'level_quad':
                self._odometry = odometry

    def _calculate_trapezoidal_acceleration(self, distance, current_velocity):
        # return two twists stamped appropriately
        max_acceleration = 1.0
        max_velocity = 1.5

        # Check if we can decelerate in time
        target_acceleration = (current_velocity**2)/(2*distance)

        if target_acceleration < max_acceleration:
            if current_velocity < max_velocity:
                target_acceleration = max_acceleration
            else:
                target_acceleration = 0
        else:
            target_acceleration = -max_acceleration

        return target_acceleration

    def _get_next_acceleration_target_trapezoidal(self, x, y, current_twist, z_velocity=None):
        # calculate current straight line velocity
        velocity = math.sqrt(current_twist.twist.linear.x**2 + current_twist.twist.linear.y**2)
        # calculate straight line distance
        distance = math.sqrt(x**2 + y**2)

        target_acceleration = self._calculate_trapezoidal_acceleration(distance, velocity)

        current_angle = math.atan2(x, y)

        first_twist = TwistStamped()
        first_twist.header.stamp = rospy.Time.now()
        first_twist.twist.linear.x = current_twist.twist.linear.x
        first_twist.twist.linear.y = current_twist.twist.linear.y

        second_twist = TwistStamped()
        second_twist.header.stamp = rospy.Time.now() + rospy.Duration(self._update_period)
        second_twist.twist.linear.x = current_twist.twist.linear.x+(target_acceleration*self._update_period*math.cos(current_angle))
        second_twist.twist.linear.y = current_twist.twist.linear.y+(target_acceleration*self._update_period*math.sin(current_angle))
        
        if z_velocity is not None:
            first_twist.twist.linear.z = z_velocity
            second_twist.twist.linear.z = z_velocity

        # Set a target accleration by setting velocity that's timestamped into the future based on the current velocity
        return (first_twist, second_twist)

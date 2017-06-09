#!/usr/bin/env python

# A helper class for a task that will spit out a velocity that's appropriate
# to reach and hold a desired position in the z dimension
# Uses a simple p-controller

import math
import rospy
import threading

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry

class HeightHolder():
    def __init__(self):
        self._odometry = None
        self._lock = threading.RLock()
        self._current_velocity_sub = rospy.Subscriber('/odometry/filtered',
                                              Odometry,
                                              self._current_velocity_callback)
        try:
            self._MIN_MANEUVER_HEIGHT = rospy.get_param('~min_maneuver_height')
            self._TRACK_HEIGHT = rospy.get_param('~track_roomba_height')
            self._K_Z = rospy.get_param('~k_term_hold_z')
        except KeyError as e:
            rospy.logerr('Could not lookup a parameter for track roomba task')
            raise

        if self._TRACK_HEIGHT < self._MIN_MANEUVER_HEIGHT:
            raise ValueError('Track Roomba height was below the minimum maneuver height')

    # uses a p-controller to return a velocity to maintain a height
    # that is set as the param _track_roomba_height
    def get_height_hold_response(self):
        with self._lock:
            if self._odometry.pose.pose.position.z  < self._MIN_MANEUVER_HEIGHT:
                raise ValueError('Drone height was below the minimum maneuver height')

            delta_z = self._TRACK_HEIGHT - self._odometry.pose.pose.position.z 
            response = self._K_Z * delta_z + self._odometry.twist.twist.linear.z
            return response

    def _current_velocity_callback(self, odometry):
        with self._lock:
            self._odometry = odometry

#!/usr/bin/env python

"""
HeightHolder: provides a simple means for a task to get a z-velocity
response that maintains a certain height above the ground

"""

import rospy
import threading
from iarc7_msgs.msg import Float64ArrayStamped

class HeightHolder(object):
    def __init__(self, desired_height = None):
        self._lock = threading.RLock()
        self._delta_z = 0
        self._DESIRED_HEIGHT = desired_height
        try:
            self._MIN_MANEUVER_HEIGHT = rospy.get_param('~min_maneuver_height')
            self._MAX_Z_ERROR = rospy.get_param('~max_z_error')
            self._K_Z = rospy.get_param('~p_term_height_hold_z')
            self._DEADZONE = rospy.get_param('~deadzone_height_hold_z')
            self._DEADZONE_HYSTERESIS = rospy.get_param('~deadzone_hysteresis_height_hold_z')
            self._DEBUG = rospy.get_param('~debug_height_hold_z')
        except KeyError as e:
            rospy.logerr('Could not lookup a parameter for track roomba task')
            raise

        if self._DEBUG:
            self._debug_pub = rospy.Publisher('~debug_height_hold',
                                              Float64ArrayStamped,
                                              queue_size=10)

        if self._DESIRED_HEIGHT is not None and self._DESIRED_HEIGHT < self._MIN_MANEUVER_HEIGHT:
            raise ValueError('Desired height was below the minimum maneuver height')
        self._deadzone_activated = False

    # uses a p-controller to return a velocity to maintain a height
    def get_height_hold_response(self, height):
        with self._lock:
            if self._DESIRED_HEIGHT is None:
                raise ValueError('No height to hold')
            self._delta_z = self._DESIRED_HEIGHT - height

            if self._DEBUG:
                msg = Float64ArrayStamped()
                msg.header.stamp = rospy.Time.now()
                msg.data = [self._DESIRED_HEIGHT, height, self._delta_z]
                self._debug_pub.publish(msg)

            # If the drone is in the deadzone area a velocity of zero is commanded
            # Essentially this is a tolerance region
            if abs(self._delta_z) < self._DEADZONE:
                self._deadzone_activated = True
            elif (self._deadzone_activated == True 
                     and abs(self._delta_z) > self._DEADZONE
                                              + self._DEADZONE_HYSTERESIS):
                self._deadzone_activated = False

            if self._deadzone_activated:
                return 0.0
            else:
                return self._K_Z * self._delta_z

    def set_height(self, desired_height):
         if desired_height < self._MIN_MANEUVER_HEIGHT:
            raise ValueError('Requested height is too low')

         self._DESIRED_HEIGHT = desired_height

    def check_z_error(self, current_height):
        return (abs(self._delta_z) < self._MAX_Z_ERROR)

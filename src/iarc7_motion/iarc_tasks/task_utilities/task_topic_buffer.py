from nav_msgs.msg import Odometry

from iarc7_msgs.msg import BoolStamped
from iarc7_msgs.msg import OdometryArray

import rospy
import tf2_ros

class TaskTopicBuffer(object):
    def __init__(self):
        self._landed_message = None
        self._roomba_array = None
        self._drone_odometry = None

        self._landing_message_sub = rospy.Subscriber(
            'landing_detected', BoolStamped, 
            self._receive_landing_status)

        self._roomba_status_sub = rospy.Subscriber(
            'roombas', OdometryArray, 
            self._receive_roomba_status)

        self._current_velocity_sub = rospy.Subscriber(
            '/odometry/filtered', Odometry,
            self._current_velocity_callback)

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)
        self._motion_profile_generator = LinearMotionProfileGenerator.get_linear_motion_profile_generator()

    def _receive_roomba_status(self, data):
        self._roomba_array = data

    def _receive_landing_status(self, data):
        self._landed_message = data

    def _current_velocity_callback(self, data):
        self._drone_odometry = data

    def has_landing_message(self):
        return self._landed_message is not None

    def has_roomba_message(self):
        return self._roomba_array is not None

    def has_odometry_message(self):
        return self._drone_odometry is not None

    def get_landing_message(self):
        return self._landed_message

    def get_roomba_message(self):
        return self._roomba_array

    def get_odometry_message(self):
        return self._drone_odometry

    def get_tf_buffer(self):
        return self._tf_buffer

    def get_linear_motion_profile_generator(self):
        return self._motion_profile_generator



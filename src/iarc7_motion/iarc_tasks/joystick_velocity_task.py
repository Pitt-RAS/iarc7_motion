#!/usr/bin/env python

import math
import rospy
import tf2_ros
import tf2_geometry_msgs
import threading

from geometry_msgs.msg import TwistStamped, PointStamped, Point

from .abstract_task import AbstractTask

from iarc_tasks.task_states import (TaskRunning,
                                    TaskDone,
                                    TaskCanceled,
                                    TaskAborted,
                                    TaskFailed)
from iarc_tasks.task_commands import (VelocityCommand,
                                      NopCommand)

from task_utilities.height_holder import HeightHolder
from task_utilities.height_settings_checker import HeightSettingsChecker
from task_utilities.acceleration_limiter import AccelerationLimiter

from sensor_msgs.msg import Joy

class JoystickVelocityTaskState(object):
    init = 0
    moving = 1
    waiting = 2

class JoystickVelocityTask(AbstractTask):

    def __init__(self, task_request):
        super(JoystickVelocityTask, self).__init__()

        self._canceled = False
        self._transition = None

        self._lock = threading.RLock()

        self._controller_sub = rospy.Subscriber(
            '/motion_joy', Joy,
            self._controller_callback)

        try:
            self._TRANSFORM_TIMEOUT = rospy.get_param('~transform_timeout')
            self._MAX_TRANSLATION_SPEED = rospy.get_param('~max_translation_speed')
            self._MAX_Z_VELOCITY = rospy.get_param('~max_z_velocity')

        except KeyError as e:
            rospy.logerr('Could not lookup a parameter for velocity task')
            raise

        self._x_vel = 0.0
        self._y_vel = 0.0
        self._z_vel = 0.0

        self._height_checker = HeightSettingsChecker()

        self._state = JoystickVelocityTaskState.init

        self._linear_motion_profile_generator = self.topic_buffer.get_linear_motion_profile_generator()

    def _controller_callback(self, data):
        if data.buttons[5]:
            self._x_vel = data.axes[1] * 1.0
            self._y_vel = data.axes[0] * 1.0
            self._z_vel = data.axes[4] * 1.0
        else:
            self._x_vel = 0.0
            self._y_vel = 0.0
            self._z_vel = 0.0

        if data.buttons[4]:
            self._canceled = True
            rospy.loginfo('JoystickVelocityTask Exiting')


    def get_desired_command(self):
        with self._lock:

            if self._canceled:
                return (TaskCanceled(),)

            if (self._state == JoystickVelocityTaskState.init):
                if not self.topic_buffer.has_odometry_message():
                    self._state = JoystickVelocityTaskState.waiting
                else:
                    self._state = JoystickVelocityTaskState.moving

            if (self._state == JoystickVelocityTaskState.waiting):
                if not self.topic_buffer.has_odometry_message():
                    return (TaskRunning(), NopCommand())
                else:
                    self._state = JoystickVelocityTaskState.moving

            if self._state == JoystickVelocityTaskState.moving:

                predicted_motion_point = self._linear_motion_profile_generator.expected_point_at_time(
                                           rospy.Time.now())

                odometry = self.topic_buffer.get_odometry_message()

                current_height = odometry.pose.pose.position.z

                if not (self._height_checker.above_min_maneuver_height(
                            current_height)):
                    return (TaskAborted(msg='Drone is too low'),)

                velocity = TwistStamped()
                velocity.header.frame_id = 'level_quad'
                velocity.header.stamp = rospy.Time.now()
                velocity.twist.linear.x = self._x_vel
                velocity.twist.linear.y = self._y_vel
                velocity.twist.linear.z = self._z_vel
                velocity_command = VelocityCommand(velocity)

                return (TaskRunning(), velocity_command)

            return (TaskAborted(msg='Illegal state reached in JoystickVelocityTask'),)

    def cancel(self):
        rospy.loginfo('JoystickVelocityTask canceled')
        self._canceled = True
        return True

    def set_incoming_transition(self, transition):
        self._transition = transition

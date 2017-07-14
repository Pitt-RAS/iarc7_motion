#!/usr/bin/env python

import math
import rospy
import tf2_ros
import tf2_geometry_msgs
import threading

from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry

from .abstract_task import AbstractTask
from iarc_tasks.task_states import (TaskRunning,
                                    TaskDone,
                                    TaskCanceled,
                                    TaskAborted,
                                    TaskFailed)
from iarc_tasks.task_commands import (VelocityCommand,
                                      ArmCommand,
                                      NopCommand)

from height_holder import HeightHolder
from height_settings_checker import HeightSettingsChecker
from acceleration_limiter import AccelerationLimiter

class VelocityTaskState(object):
    init = 0
    moving = 1
    waiting = 2

class VelocityTask(object, AbstractTask):

    def __init__(self, task_request):

        self._drone_odometry = None
        self._canceled = False

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)  

        self._lock = threading.RLock()

        self._current_velocity_sub = rospy.Subscriber(
            '/odometry/filtered', Odometry,
            self._current_velocity_callback)

        try:
            self._TRANSFORM_TIMEOUT = rospy.get_param('~transform_timeout')
            self._MAX_TRANSLATION_SPEED = rospy.get_param('~max_translation_speed')
            self._MAX_Z_VELOCITY = rospy.get_param('~max_z_velocity')
            self._HORIZ_X_VEL = rospy.get_param('~velocity_x')
            self._HORIZ_Y_VEL = rospy.get_param('~velocity_y')
            _DESIRED_HEIGHT = rospy.get_param('~velocity_task_height')

        except KeyError as e:
            rospy.logerr('Could not lookup a parameter for velocity task')
            raise

        self._z_holder = HeightHolder(_DESIRED_HEIGHT)
        self._height_checker = HeightSettingsChecker()
        self._limiter = AccelerationLimiter()

        self._state = VelocityTaskState.init

    def _current_velocity_callback(self, data):
        with self._lock:
            self._drone_odometry = data

    def get_desired_command(self):
        with self._lock:
            if (self._state == VelocityTaskState.init):
                if self._drone_odometry is None:
                    self._state = VelocityTaskState.waiting
                else:
                    self._state = VelocityTaskState.moving
                return (TaskRunning(), NopCommand())

            elif (self._state == VelocityTaskState.waiting):
                if self._drone_odometry is None:
                    self._state = VelocityTaskState.waiting
                else:
                    self._state = VelocityTaskState.moving
                return (TaskRunning(), NopCommand())

            elif self._canceled:
                return (TaskCanceled(),)

            elif self._state == VelocityTaskState.moving:

                if not (self._height_checker.above_min_maneuver_height(
                            self._drone_odometry.pose.pose.position.z)):
                    return (TaskAborted(msg='Drone is too low'),)

                x_vel_target = self._HORIZ_X_VEL
                y_vel_target = self._HORIZ_Y_VEL
                z_vel_target = self._z_holder.get_height_hold_response(
                    self._drone_odometry.pose.pose.position.z)

                if (abs(z_vel_target) > self._MAX_Z_VELOCITY):
                    z_vel_target = z_vel_target/abs(z_vel_target) * self._MAX_Z_VELOCITY

                desired_vel = []
                desired_vel.append(x_vel_target)
                desired_vel.append(y_vel_target)
                desired_vel.append(z_vel_target)

                curr_vel = []
                curr_vel.append(self._drone_odometry.twist.twist.linear.x)
                curr_vel.append(self._drone_odometry.twist.twist.linear.y)
                curr_vel.append(self._drone_odometry.twist.twist.linear.z)

                desired_vel = self._limiter.limit_acceleration(curr_vel, desired_vel)

                velocity = TwistStamped()
                velocity.header.frame_id = 'level_quad'
                velocity.header.stamp = rospy.Time.now()
                velocity.twist.linear.x = desired_vel[0]
                velocity.twist.linear.y = desired_vel[1]
                velocity.twist.linear.z = desired_vel[2]
                
                return (TaskRunning(), VelocityCommand(velocity)) 
            else:
                return (TaskAborted(msg='Illegal state reached in Velocity test task'),)

    def cancel(self):
        rospy.loginfo('VelocityTestTask Task canceled')
        self._canceled = True

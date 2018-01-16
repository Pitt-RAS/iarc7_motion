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

class VelocityTaskState(object):
    init = 0
    moving = 1
    waiting = 2

class VelocityTask(AbstractTask):

    def __init__(self, task_request):
        super(VelocityTask, self).__init__()

        self._canceled = False
        self._current_velocity = None
        self._current_height_set = False
        self._transition = None

        self._lock = threading.RLock()

        try:
            self._TRANSFORM_TIMEOUT = rospy.get_param('~transform_timeout')
            self._MAX_TRANSLATION_SPEED = rospy.get_param('~max_translation_speed')
            self._MAX_Z_VELOCITY = rospy.get_param('~max_z_velocity')

        except KeyError as e:
            rospy.logerr('Could not lookup a parameter for velocity task')
            raise

        self._HORIZ_X_VEL = task_request.x_velocity
        self._HORIZ_Y_VEL = task_request.y_velocity

        self._z_holder = HeightHolder()
        self._height_checker = HeightSettingsChecker()
        self._limiter = AccelerationLimiter()

        self._state = VelocityTaskState.init

    def get_desired_command(self):
        with self._lock:

            if self._canceled:
                return (TaskCanceled(),)

            if (self._state == VelocityTaskState.init):
                if not self.topic_buffer.has_odometry_message():
                    self._state = VelocityTaskState.waiting
                else:
                    self._state = VelocityTaskState.moving

            if (self._state == VelocityTaskState.waiting):
                if not self.topic_buffer.has_odometry_message():
                    return (TaskRunning(), NopCommand())
                else:
                    self._state = VelocityTaskState.moving

            if self._state == VelocityTaskState.moving:

                try:
                    transStamped = self.topic_buffer.get_tf_buffer().lookup_transform(
                                    'map',
                                    'base_footprint',
                                    rospy.Time(0),
                                    rospy.Duration(self._TRANSFORM_TIMEOUT))
                except (tf2_ros.LookupException,
                        tf2_ros.ConnectivityException,
                        tf2_ros.ExtrapolationException) as ex:
                    rospy.logerr('ObjectTrackTask: Exception when looking up transform')
                    rospy.logerr(ex.message)
                    return (TaskAborted(msg='Exception when looking up transform during velocity task'),)

                current_height = transStamped.transform.translation.z

                if not self._current_height_set:
                    self._current_height_set = True
                    self._z_holder.set_height(current_height)

                if not (self._height_checker.above_min_maneuver_height(
                            current_height)):
                    return (TaskAborted(msg='Drone is too low'),)

                x_vel_target = self._HORIZ_X_VEL
                y_vel_target = self._HORIZ_Y_VEL
                z_vel_target = self._z_holder.get_height_hold_response(current_height)

                if (abs(z_vel_target) > self._MAX_Z_VELOCITY):
                    z_vel_target = math.copysign(self._MAX_Z_VELOCITY, z_vel_target)

                desired_vel = [x_vel_target, y_vel_target, z_vel_target]

                odometry = self.topic_buffer.get_odometry_message()
                drone_vel_x = odometry.twist.twist.linear.x
                drone_vel_y = odometry.twist.twist.linear.y
                drone_vel_z = odometry.twist.twist.linear.z

                if self._current_velocity is None:
                    self._current_velocity = [drone_vel_x, drone_vel_y, drone_vel_z]

                desired_vel = self._limiter.limit_acceleration(self._current_velocity, desired_vel)

                velocity = TwistStamped()
                velocity.header.frame_id = 'level_quad'
                velocity.header.stamp = rospy.Time.now()
                velocity.twist.linear.x = desired_vel[0]
                velocity.twist.linear.y = desired_vel[1]
                velocity.twist.linear.z = desired_vel[2]

                self._current_velocity = desired_vel

                return (TaskRunning(), VelocityCommand(velocity))

            return (TaskAborted(msg='Illegal state reached in Velocity test task'),)

    def cancel(self):
        rospy.loginfo('VelocityTestTask Task canceled')
        self._canceled = True
        return True

    def set_incoming_transition(self, transition):
        self._transition = transition

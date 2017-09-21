#!/usr/bin/env python

import math
import rospy
import tf2_ros
import tf2_geometry_msgs
import threading
import numpy as np

from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Vector3Stamped
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry

from acceleration_limiter import AccelerationLimiter 

from iarc7_msgs.msg import OdometryArray
from iarc7_msgs.msg import LandingGearContactsStamped

from .abstract_task import AbstractTask
from iarc_tasks.task_states import (TaskRunning,
                                    TaskDone,
                                    TaskCanceled,
                                    TaskAborted,
                                    TaskFailed)
from iarc_tasks.task_commands import (VelocityCommand,
                                      NopCommand)

class BlockRoombaTaskState:
    init = 0
    waiting = 1
    descent = 2

class BlockRoombaTask(AbstractTask):

    def __init__(self, task_request):

        self._roomba_id = task_request.frame_id

        self._roomba_id = self._roomba_id  + '/base_link'

        if self._roomba_id is None:
            raise ValueError('A null roomba id was provided')

        self._roomba_odometry = None
        self._roomba_array = None
        self._roomba_point = None
        self._limiter = AccelerationLimiter()

        self._drone_odometry = None
        self._canceled = False
        self._switch_message = None
        self._last_update_time = None
        self._current_velocity = None

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)  

        self._lock = threading.RLock()

        self._contact_switches_sub = rospy.Subscriber(
            'landing_gear_contact_switches', LandingGearContactsStamped, 
            self._receive_switch_status)

        self._roomba_status_sub = rospy.Subscriber(
            'roombas', OdometryArray, 
            self._receive_roomba_status)

        self._current_velocity_sub = rospy.Subscriber(
            '/odometry/filtered', Odometry,
            self._current_velocity_callback)

        try:
            self._TRANSFORM_TIMEOUT = rospy.get_param('~transform_timeout')
            self._MIN_MANEUVER_HEIGHT = rospy.get_param('~min_maneuver_height')
            self._MAX_TRANSLATION_SPEED = rospy.get_param('~max_translation_speed')
            self._MAX_START_TASK_DIST = rospy.get_param('~hit_roomba_max_start_dist')
            self._MAX_Z_VELOCITY = rospy.get_param('~max_z_velocity')
            self._K_X = rospy.get_param('~k_term_tracking_x')
            self._K_Y = rospy.get_param('~k_term_tracking_y')
            self._descent_velocity = rospy.get_param('~hit_descent_velocity')
            _roomba_diameter = rospy.get_param('~roomba_diameter')
            _drone_width = rospy.get_param('~drone_landing_gear_width') 
        except KeyError as e:
            rospy.logerr('Could not lookup a parameter for block roomba task')
            raise
        self._overshoot = (_roomba_diameter + _drone_width)/2
        self._state = BlockRoombaTaskState.init

    def _receive_roomba_status(self, data):
        with self._lock:
            self._roomba_array = data

    def _receive_switch_status(self, data):
        with self._lock:
            self._switch_message = data

    def _current_velocity_callback(self, data):
        with self._lock:
            self._drone_odometry = data

    def get_desired_command(self):
        with self._lock:
            if self._canceled:
                return (TaskCanceled(),)

            elif (self._state == BlockRoombaTaskState.init):
                if self._roomba_array is None or self._drone_odometry is None:
                    self._state = BlockRoombaTaskState.waiting
                else:
                    self._state = BlockRoombaTaskState.descent
                return (TaskRunning(), NopCommand())

            elif (self._state == BlockRoombaTaskState.waiting):
                if self._roomba_array is None or self._drone_odometry is None:
                    self._state = BlockRoombaTaskState.waiting
                else:
                    self._state = BlockRoombaTaskState.descent
                return (TaskRunning(), NopCommand())

            elif (self._state == BlockRoombaTaskState.descent):
                try:
                    roomba_transform = self._tf_buffer.lookup_transform(
                                        'level_quad',
                                        self._roomba_id,
                                        rospy.Time.now(),
                                        rospy.Duration(self._TRANSFORM_TIMEOUT))
                    drone_transform = self._tf_buffer.lookup_transform(
                                        'level_quad',
                                        'map',
                                        rospy.Time.now(),
                                        rospy.Duration(self._TRANSFORM_TIMEOUT))
                except (tf2_ros.LookupException,
                        tf2_ros.ConnectivityException,
                        tf2_ros.ExtrapolationException) as ex:
                    rospy.logerr('Block Roomba Task: Exception when looking up transform')
                    rospy.logerr(ex.message)
                    return (TaskAborted(msg='Exception when looking up transform during block roomba'),)

                # Creat point centered at drone's center
                stamped_point = PointStamped()
                stamped_point.point.x = 0
                stamped_point.point.y = 0
                stamped_point.point.z = 0

                # returns point distances of roomba to center point of level quad
                self._roomba_point = tf2_geometry_msgs.do_transform_point(
                                                    stamped_point, roomba_transform)

                if not self._check_max_roomba_range():
                    return (TaskAborted(msg='The provided roomba is not close enough to the quad'),)
                if self._on_ground():
                    return (TaskDone(),)

                roomba_x_velocity = self._roomba_odometry.twist.twist.linear.x
                roomba_y_velocity = self._roomba_odometry.twist.twist.linear.y
                roomba_velocity = math.sqrt(roomba_x_velocity**2 + roomba_y_velocity**2)

                roomba_vector = Vector3Stamped()

                roomba_vector.vector.x = roomba_x_velocity/roomba_velocity
                roomba_vector.vector.y = roomba_y_velocity/roomba_velocity
                roomba_vector.vector.z = 0.0

                # p-controller
                x_vel_target = ((self._roomba_point.point.x + self._overshoot * roomba_vector.vector.x)
                                    * self._K_X + roomba_x_velocity)
                y_vel_target = ((self._roomba_point.point.y + self._overshoot * roomba_vector.vector.y)
                                    * self._K_Y + roomba_y_velocity)
                
                z_vel_target = self._descent_velocity

                #caps velocity
                vel_target = math.sqrt(x_vel_target**2 + y_vel_target**2)

                if vel_target > self._MAX_TRANSLATION_SPEED:
                    x_vel_target = x_vel_target * (self._MAX_TRANSLATION_SPEED/vel_target)
                    y_vel_target = y_vel_target * (self._MAX_TRANSLATION_SPEED/vel_target)
                
                if (abs(z_vel_target) > self._MAX_Z_VELOCITY):
                    z_vel_target = z_vel_target/abs(z_vel_target) * self._MAX_Z_VELOCITY
                    rospy.logwarn("Max Z velocity reached in block roomba")

                desired_vel = [x_vel_target, y_vel_target, z_vel_target]
               
                drone_vel_x = self._drone_odometry.twist.twist.linear.x
                drone_vel_y = self._drone_odometry.twist.twist.linear.y
                drone_vel_z = self._drone_odometry.twist.twist.linear.z

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

            else:
                return (TaskAborted(msg='Illegal state reached in Block Roomba Task' ),)

    # checks to see if passed in roomba id is available and
    # that the drone and roomba are both within a specified distance
    # in order to start/continue the task
    def _check_max_roomba_range(self):
        for odometry in self._roomba_array.data:
            if odometry.child_frame_id == self._roomba_id:
                self._roomba_odometry = odometry
                self._roomba_found =  True 
                _distance_to_roomba = math.sqrt(self._roomba_point.point.x**2 + 
                            self._roomba_point.point.y**2)
                return (_distance_to_roomba <= (self._MAX_START_TASK_DIST + self._overshoot))
        return False

    def cancel(self):
        rospy.loginfo('HitRoomba Task canceled')
        self._canceled = True

    def _on_ground(self):
        if self._switch_message is None:
            return False
        else: 
            data = self._switch_message
            return (data.front or data.back or data.left or data.right)

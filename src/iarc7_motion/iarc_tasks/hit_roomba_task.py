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
                                      ArmCommand,
                                      NopCommand)

class HitRoombaTaskState(object):
    init = 0
    waiting = 1
    descent = 2

class HitRoombaTask(object, AbstractTask):

    def __init__(self, task_request):

        self._roomba_id = task_request.frame_id  + '/base_link'

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
            self._MAX_HORIZ_SPEED = rospy.get_param('~max_translation_speed')
            self._MAX_START_TASK_DIST = rospy.get_param('~hit_roomba_max_start_dist')
            self._MAX_Z_VELOCITY = rospy.get_param('~max_z_velocity')
            self._K_X = rospy.get_param('~k_term_tracking_x')
            self._K_Y = rospy.get_param('~k_term_tracking_y')
            self._descent_velocity = rospy.get_param('~hit_descent_velocity')
        except KeyError as e:
            rospy.logerr('Could not lookup a parameter for hit roomba task')
            raise

        self._state = HitRoombaTaskState.init

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

            elif (self._state == HitRoombaTaskState.init):
                if self._roomba_array is None or self._drone_odometry is None:
                    self._state = HitRoombaTaskState.waiting
                else:
                    self._state = HitRoombaTaskState.descent
                return (TaskRunning(), NopCommand())

            elif (self._state == HitRoombaTaskState.waiting):
                if self._roomba_array is None or self._drone_odometry is None:
                    self._state = HitRoombaTaskState.waiting
                else:
                    self._state = HitRoombaTaskState.descent
                return (TaskRunning(), NopCommand())

            elif (self._state == HitRoombaTaskState.descent):

                if not self._check_roomba_in_sight():
                    return (TaskAborted(msg='The provided roomba is not in sight of quad'),)

                elif self._on_ground():
                    return (TaskDone(),)

                try:
                    roomba_transform = self._tf_buffer.lookup_transform(
                                        'level_quad',
                                        self._roomba_id,
                                        rospy.Time.now(),
                                        rospy.Duration(self._TRANSFORM_TIMEOUT))
                except (tf2_ros.LookupException,
                        tf2_ros.ConnectivityException,
                        tf2_ros.ExtrapolationException) as ex:
                    rospy.logerr('ObjectTrackTask: Exception when looking up transform')
                    rospy.logerr(ex.message)
                    return (TaskAborted(msg='Exception when looking up transform during hit roomba'),)

                # Creat point centered at drone's center
                stamped_point = PointStamped()
                stamped_point.point.x = 0
                stamped_point.point.y = 0
                stamped_point.point.z = 0

                # returns point distances of roomba to center point of level quad
                self._roomba_point = tf2_geometry_msgs.do_transform_point(
                                                    stamped_point, roomba_transform)

                # uses _roomba_point to determine if quad too far from roomba
                if not self._check_max_roomba_range():
                    return (TaskAborted(msg='The provided roomba is not close enough to the quad'),)
                
                roomba_x_velocity = self._roomba_odometry.twist.twist.linear.x
                roomba_y_velocity = self._roomba_odometry.twist.twist.linear.y

                # p-controller
                x_vel_target = (self._roomba_point.point.x * self._K_X + roomba_x_velocity)
                y_vel_target = (self._roomba_point.point.y * self._K_Y + roomba_y_velocity)
                
                z_vel_target = self._descent_velocity

                #caps velocity
                vel_target = math.sqrt(x_vel_target**2 + y_vel_target**2)

                if vel_target > self._MAX_HORIZ_SPEED:
                    x_vel_target = x_vel_target * (self._MAX_HORIZ_SPEED/vel_target)
                    y_vel_target = y_vel_target * (self._MAX_HORIZ_SPEED/vel_target)
                
                if (abs(z_vel_target) > self._MAX_Z_VELOCITY):
                    z_vel_target = math.copysign(self._MAX_Z_VELOCITY, z_vel_target)

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

            return (TaskAborted(msg='Illegal state reached in Hit Roomba Task' ),)

    # checks to see if passed in roomba id is in sight of quad
    def _check_roomba_in_sight(self):
        for odometry in self._roomba_array.data:
            if odometry.child_frame_id == self._roomba_id:
                self._roomba_odometry = odometry
                return True
        return False

    # that the drone and roomba are both within a specified distance
    # in order to start/continue the task
    def _check_max_roomba_range(self):
        _distance_to_roomba = math.sqrt(self._roomba_point.point.x**2 + 
                            self._roomba_point.point.y**2)
        
        return (_distance_to_roomba <= self._MAX_START_TASK_DIST)

    def cancel(self):
        rospy.loginfo('HitRoomba Task canceled')
        self._canceled = True

    def _on_ground(self):
        if self._switch_message is None:
            return False
        else: 
            data = self._switch_message
            return (data.front or data.back or data.left or data.right)
   
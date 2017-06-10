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


from iarc7_msgs.msg import OdometryArray

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
from height_checker import HeightChecker

class TrackObjectTaskState:
    init = 0
    track = 1
    waiting = 2

class TrackRoombaTask(AbstractTask):

    def __init__(self, task_dictionary):

        self._roomba_id = task_dictionary['frame_id']

        self._roomba_id = self._roomba_id  + '/base_link'

        if self._roomba_id is None:
            raise ValueError('A null roomba id was provided')

        self._roomba_odometry = None
        self._roomba_array = None
        self._roomba_point = None
        self._roomba_found = False

        self._drone_odometry = None
        self._canceled = False

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)  

        self._lock = threading.RLock()

        self._roomba_status_sub = rospy.Subscriber(
            'roombas', OdometryArray, 
            self._receive_roomba_status)

        self._current_velocity_sub = rospy.Subscriber(
            '/odometry/filtered', Odometry,
            self._current_velocity_callback)

        try:
            self._TRANSFORM_TIMEOUT = rospy.get_param('~transform_timeout')
            self._MAX_TRANSLATION_SPEED = rospy.get_param('~max_translation_speed')
            self._MAX_START_TASK_DIST = rospy.get_param('~roomba_max_start_task_dist')
            self._MAX_END_TASK_DIST = rospy.get_param('~roomba_max_end_task_dist')
            self._MAX_Z_VELOCITY = rospy.get_param('~max_z_velocity')
            self._K_X = rospy.get_param('~k_term_tracking_x')
            self._K_Y = rospy.get_param('~k_term_tracking_y')
        except KeyError as e:
            rospy.logerr('Could not lookup a parameter for track roomba task')
            raise

        self._z_holder = HeightHolder()
        self._height_checker = HeightChecker()

        self._state = TrackObjectTaskState.init

    def _receive_roomba_status(self, data):
        with self._lock:
            self._roomba_array = data

    def _current_velocity_callback(self, data):
        with self._lock:
            self._drone_odometry = data

    def get_desired_command(self):
        with self._lock:
            if (self._state != TrackObjectTaskState.track):
                if self._roomba_array is None or self._drone_odometry is None:
                    self._state = TrackObjectTaskState.waiting
                    return (TaskRunning(), NopCommand())
                else:
                    self._state = TrackObjectTaskState.track
            elif not (self._height_checker.above_min_maneuver_height(self._drone_odometry.pose.pose.position.z)):
                return (TaskAborted(msg='Drone is too low'),)

            if self._canceled:
                return (TaskCanceled(),)

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
                return (TaskAborted(msg='Exception when looking up transform during roomba track'),)

            # Creat point centered at drone's center
            stamped_point = PointStamped()
            stamped_point.point.x = 0
            stamped_point.point.y = 0
            stamped_point.point.z = 0

            # returns point distances of roomba to center point of level quad
            self._roomba_point = tf2_geometry_msgs.do_transform_point(
                                                stamped_point, roomba_transform)

            if not self._check_max_start_roomba_range():
                return (TaskAborted(msg='The provided roomba is not found or not within a meter of the quad'),)

            if self._check_max_ending_roomba_range():
                return (TaskDone(),)

            # p-controller
            x_vel_target = (self._roomba_point.point.x * self._K_X + 
                        self._roomba_odometry.twist.twist.linear.x)
            y_vel_target = (self._roomba_point.point.y * self._K_Y + 
                        self._roomba_odometry.twist.twist.linear.y)
            z_vel_target = self._z_holder.get_height_hold_response(
                self._drone_odometry.pose.pose.position.z,
                self._drone_odometry.twist.twist.linear.z)

            #caps velocity
            vel_target = math.sqrt(x_vel_target**2 + y_vel_target**2)

            if vel_target > self._MAX_TRANSLATION_SPEED:
                x_vel_target = x_vel_target * (self._MAX_TRANSLATION_SPEED/vel_target)
                y_vel_target = y_vel_target * (self._MAX_TRANSLATION_SPEED/vel_target)
            
            if (abs(z_vel_target) > self._MAX_Z_VELOCITY):
                z_vel_target = z_vel_target/abs(z_vel_target) * self._MAX_Z_VELOCITY

            velocity = TwistStamped()
            velocity.header.frame_id = 'level_quad'
            velocity.header.stamp = rospy.Time.now()
            velocity.twist.linear.x = x_vel_target
            velocity.twist.linear.y = y_vel_target
            velocity.twist.linear.z = z_vel_target
            
            return (TaskRunning(), VelocityCommand(velocity)) 

    # checks to see if passed in roomba id is available and
    # that the drone and roomba are both within a specified distance
    # in order to start the task
    def _check_max_start_roomba_range(self):
        for odometry in self._roomba_array.data:
            if odometry.child_frame_id == self._roomba_id:
                self._roomba_odometry = odometry
                self._roomba_found =  True 
                _distance_to_roomba = math.sqrt(self._roomba_point.point.x**2 + 
                            self._roomba_point.point.y**2)
                return (_distance_to_roomba <= self._MAX_START_TASK_DIST)
        return False

    # checks the radial distance from the drone to the roomba
    # in order to end the task as successful
    def _check_max_ending_roomba_range(self):
        _distance_to_roomba = math.sqrt(self._roomba_point.point.x**2 + 
                            self._roomba_point.point.y**2)
        
        return (_distance_to_roomba <= self._MAX_END_TASK_DIST)

    def cancel(self):
        rospy.loginfo('TrackRoomba Task canceled')
        self._canceled = True

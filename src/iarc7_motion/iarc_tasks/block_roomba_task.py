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

from task_utilities.acceleration_limiter import AccelerationLimiter

from .abstract_task import AbstractTask
from iarc_tasks.task_states import (TaskRunning,
                                    TaskDone,
                                    TaskCanceled,
                                    TaskAborted,
                                    TaskFailed)
from iarc_tasks.task_commands import (VelocityCommand,
                                      NopCommand)

class BlockRoombaTaskState(object):
    init = 0
    waiting = 1
    descent = 2

class BlockRoombaTask(AbstractTask):

    def __init__(self, task_request):
        super(BlockRoombaTask, self).__init__()

        self._roomba_id = task_request.frame_id

        self._roomba_id = self._roomba_id  + '/base_link'

        if self._roomba_id is None:
            raise ValueError('A null roomba id was provided')

        self._roomba_odometry = None
        self._roomba_point = None
        self._limiter = AccelerationLimiter()

        self._canceled = False
        self._last_update_time = None
        self._current_velocity = None
        self._transition = None

        self._lock = threading.RLock()

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

    def get_desired_command(self):
        with self._lock:
            if self._canceled:
                return (TaskCanceled(),)

            if (self._state == BlockRoombaTaskState.init):
                if (not self.topic_buffer.has_roomba_message()
                 or not self.topic_buffer.has_odometry_message()
                 or not self.topic_buffer.has_landing_message()):
                    self._state = BlockRoombaTaskState.waiting
                else:
                    self._state = BlockRoombaTaskState.descent

            if (self._state == BlockRoombaTaskState.waiting):
                if (not self.topic_buffer.has_roomba_message()
                 or not self.topic_buffer.has_odometry_message()
                 or not self.topic_buffer.has_landing_message()):
                    return (TaskRunning(), NopCommand())
                else:
                    self._state = BlockRoombaTaskState.descent

            if (self._state == BlockRoombaTaskState.descent):
                try:
                    roomba_transform = self.topic_buffer.get_tf_buffer().lookup_transform(
                                        'level_quad',
                                        self._roomba_id,
                                        rospy.Time(0),
                                        rospy.Duration(self._TRANSFORM_TIMEOUT))
                    drone_transform = self.topic_buffer.get_tf_buffer().lookup_transform(
                                        'level_quad',
                                        'map',
                                        rospy.Time(0),
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

            return (TaskAborted(msg='Illegal state reached in Block Roomba Task' ),)

    # checks to see if passed in roomba id is available and
    # that the drone and roomba are both within a specified distance
    # in order to start/continue the task
    def _check_max_roomba_range(self):
        for odometry in self.topic_buffer.get_roomba_message().data:
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
        return True

    def _on_ground(self):
        return self.topic_buffer.get_landing_message().data

    def set_incoming_transition(self, transition):
        self._transition = transition

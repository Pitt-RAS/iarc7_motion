#!/usr/bin/env python

import math
import rospy
import tf2_ros
import tf2_geometry_msgs
import threading

from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import PointStamped
from iarc7_msgs.msg import MotionPointStamped, MotionPointStampedArray

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
    INIT = 0
    WAITING = 1
    PLANNING = 2
    DESCENDING = 3
    LANDED = 4

class BlockRoombaTask(AbstractTask):
    def __init__(self, task_request):
        super(BlockRoombaTask, self).__init__()

        self._roomba_id = task_request.frame_id

        if self._roomba_id == '':
            raise ValueError('An invalid roomba id was provided to BlockRoombaTask')

        self._distance_to_roomba = None
        self._roomba_odometry = None
        self._roomba_point = None
        self._limiter = AccelerationLimiter()

        self._canceled = False
        self._current_velocity = None
        self._transition = None

        self._target = (0,0)

        self._lock = threading.RLock()

        try:
            self._TRANSFORM_TIMEOUT = rospy.get_param('~transform_timeout')
            self._MAX_TRANSLATION_SPEED = rospy.get_param('~max_translation_speed')
            self._MAX_START_TASK_DIST = rospy.get_param('~block_roomba_max_start_dist')
            self._MAX_Z_VELOCITY = rospy.get_param('~max_z_velocity')
            self._K_X = rospy.get_param('~k_term_tracking_x')
            self._K_Y = rospy.get_param('~k_term_tracking_y')
            self._DESCENT_VELOCITY = rospy.get_param('~block_descent_velocity')

            _roomba_diameter = rospy.get_param('~roomba_diameter')
            _drone_width = rospy.get_param('~drone_landing_gear_width')
        except KeyError as e:
            rospy.logerr('Could not lookup a parameter for block roomba task')
            raise

        # the roomba diameter/2 (radius) is acting as a buffer
        self._overshoot = _roomba_diameter + _drone_width/2
        self._state = BlockRoombaTaskState.INIT

    def get_desired_command(self):
        with self._lock:
            if self._canceled:
                return (TaskCanceled(),)

            if self._state == BlockRoombaTaskState.INIT:
                if (not self.topic_buffer.has_roomba_message()
                 or not self.topic_buffer.has_odometry_message()
                 or not self.topic_buffer.has_landing_message()):
                    self._state = BlockRoombaTaskState.WAITING
                else:
                    self._state = BlockRoombaTaskState.PLANNING

            if self._state == BlockRoombaTaskState.WAITING:
                if (not self.topic_buffer.has_roomba_message()
                 or not self.topic_buffer.has_odometry_message()
                 or not self.topic_buffer.has_landing_message()):
                    return (TaskRunning(), NopCommand())
                else:
                    self._state = BlockRoombaTaskState.PLANNING

            if self._state == BlockRoombaTaskState.PLANNING:
                try:
                    roomba_transform = self.topic_buffer.get_tf_buffer().lookup_transform(
                                        'level_quad',
                                        self._roomba_id,
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

                drone_height = self.topic_buffer.get_odometry_message().pose.pose.position.z
                time_to_descend = drone_height/self._DESCENT_VELOCITY

                roomba_vector_x = roomba_x_velocity/roomba_velocity
                roomba_vector_y = roomba_y_velocity/roomba_velocity

                goal_x = (self._roomba_point.point.x + self._overshoot * roomba_vector_x)/time_to_descend
                goal_y = (self._roomba_point.point.y + self._overshoot * roomba_vector_y)/time_to_descend

                # we need a better way to plan
                self._target = (goal_x, goal_y)

                self._state = BlockRoombaTaskState.DESCENDING

            if self._state == BlockRoombaTaskState.DESCENDING:

                x_vel_target = self._target[0]
                y_vel_target = self._target[1]

                z_vel_target = self._DESCENT_VELOCITY

                # cap velocity
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
                self._distance_to_roomba = math.sqrt(self._roomba_point.point.x**2 +
                            self._roomba_point.point.y**2)
                return (self._distance_to_roomba <= (self._MAX_START_TASK_DIST + self._overshoot))
        return False

    def cancel(self):
        rospy.loginfo('BlockRoombaTask canceled')
        self._canceled = True
        return True

    def _on_ground(self):
        return self.topic_buffer.get_landing_message().data

    def set_incoming_transition(self, transition):
        self._transition = transition

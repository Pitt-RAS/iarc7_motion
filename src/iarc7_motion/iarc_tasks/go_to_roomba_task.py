#!/usr/bin/env python

import math
import rospy
import tf2_ros
import tf2_geometry_msgs
import threading

from geometry_msgs.msg import TwistStamped, PointStamped

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

class GoToTaskState(object):
    INIT = 0
    GOING = 1
    WAITING = 2

class GoToRoombaTask(AbstractTask):

    def __init__(self, task_request):
        super(TrackRoombaTask, self).__init__()

        self._roomba_id = task_request.frame_id  + '/base_link'
        
        self._ending_radius = task_request.ending_radius

        if self._roomba_id is None:
            raise ValueError('A null roomba id was provided to GoToRoombaTask')

        self._roomba_odometry = None
        self._roomba_point = None
        self._roomba_found = False
        self._current_velocity = None
        self._transition = None

        self._canceled = False

        self._lock = threading.RLock()

        try:
            self._TRANSFORM_TIMEOUT = rospy.get_param('~transform_timeout')
            self._MAX_HORIZ_SPEED = rospy.get_param('~max_translation_speed')
            self._MAX_Z_VELOCITY = rospy.get_param('~max_z_velocity')
            TRACK_HEIGHT = rospy.get_param('~track_roomba_height')
        except KeyError as e:
            rospy.logerr('Could not lookup a parameter for track roomba task')
            raise

        self._z_holder = HeightHolder(TRACK_HEIGHT)
        self._height_checker = HeightSettingsChecker()
        self._limiter = AccelerationLimiter()

        self._state = GoToTaskState.INIT

    def get_desired_command(self):
        with self._lock:

            if self._canceled:
                return (TaskCanceled(),)

            if (self._state == GoToTaskState.INIT):
                if (not self.topic_buffer.has_roomba_message()
                 or not self.topic_buffer.has_odometry_message()):
                    self._state = GoToTaskState.WAITING
                else:
                    self._state = GoToTaskState.GOING

            if (self._state == GoToTaskState.WAITING):
                if (not self.topic_buffer.has_roomba_message()
                 or not self.topic_buffer.has_odometry_message()):
                    return (TaskRunning(), NopCommand())
                else:
                    self._state = GoToTaskState.GOING

            if self._state == GoToTaskState.GOING:
                if not (self._height_checker.above_min_maneuver_height(
                        self.topic_buffer.get_odometry_message().pose.pose.position.z)):
                    return (TaskAborted(msg='Drone is too low'),)
                elif not (self._z_holder.check_z_error(
                        self.topic_buffer.get_odometry_message().pose.pose.position.z)):
                    return (TaskAborted(msg='Z error is too high'),)
                elif not self._check_roomba_in_sight():
                    return (TaskAborted(msg='The provided roomba is not in sight of quad'),)
                try:
                    roomba_transform = self.topic_buffer.get_tf_buffer().lookup_transform(
                                        'level_quad',
                                        self._roomba_id,
                                        rospy.Time(0),
                                        rospy.Duration(self._TRANSFORM_TIMEOUT))
                except (tf2_ros.LookupException,
                        tf2_ros.ConnectivityException,
                        tf2_ros.ExtrapolationException) as ex:
                    rospy.logerr('GoToRoombaTask: Exception when looking up transform')
                    rospy.logerr(ex.message)
                    return (TaskAborted(msg='Exception when looking up transform during GoToRoombaTask'),)

                # Creat point centered at drone's center
                stamped_point = PointStamped()
                stamped_point.point.x = 0
                stamped_point.point.y = 0
                stamped_point.point.z = 0

                # returns point distances of roomba to center point of level quad
                self._roomba_point = tf2_geometry_msgs.do_transform_point(
                                                    stamped_point, roomba_transform)

                roomba_x_velocity = self._roomba_odometry.twist.twist.linear.x
                roomba_y_velocity = self._roomba_odometry.twist.twist.linear.y
                roomba_velocity = math.sqrt(roomba_x_velocity**2 + roomba_y_velocity**2)

                # p-controller
                x_vel_target = (self._roomba_point.point.x
                                    * self._K_X + roomba_x_velocity)
                y_vel_target = (self._roomba_point.point.y
                                    * self._K_Y + roomba_y_velocity)
                
                z_vel_target = self._z_holder.get_height_hold_response(
                    self.topic_buffer.get_odometry_message().pose.pose.position.z)

                # caps velocity
                vel_target = math.sqrt(x_vel_target**2 + y_vel_target**2)

                if vel_target > self._MAX_HORIZ_SPEED:
                    x_vel_target = x_vel_target * (self._MAX_HORIZ_SPEED/vel_target)
                    y_vel_target = y_vel_target * (self._MAX_HORIZ_SPEED/vel_target)
                
                if (abs(z_vel_target) > self._MAX_Z_VELOCITY):
                    z_vel_target =  math.copysign(self._MAX_Z_VELOCITY, z_vel_target)

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
            
            return (TaskAborted(msg='Unknown state reached in GoToRoombaTask'),)

    # checks to see if passed in roomba id is in sight of quad
    def _check_roomba_in_sight(self):
        for odometry in self.topic_buffer.get_roomba_message().data:
            if odometry.child_frame_id == self._roomba_id:
                self._roomba_odometry = odometry
                return True
        return False

    # checks that the drone and roomba are both within a specified distance
    # in order to end the task
    def _check_end_roomba_range(self):
        _distance_to_roomba = math.sqrt(self._roomba_point.point.x**2 + 
                            self._roomba_point.point.y**2)
        return (_distance_to_roomba <= self._ending_radius)

    def cancel(self):
        rospy.loginfo('GoToRoomba Task canceled')
        self._canceled = True
        return True
    
    def set_incoming_transition(self, transition):
        self._transition = transition

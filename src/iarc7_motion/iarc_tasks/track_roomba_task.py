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

from task_utilities.pid_controller import PidSettings, PidController
from task_utilities.height_holder import HeightHolder
from task_utilities.height_settings_checker import HeightSettingsChecker
from task_utilities.acceleration_limiter import AccelerationLimiter

class TrackRoombaTaskState(object):
    init = 0
    track = 1
    waiting = 2

class TrackRoombaTask(AbstractTask):

    def __init__(self, task_request):
        super(TrackRoombaTask, self).__init__()

        # id of the roomba we are tracking
        self._roomba_id = task_request.frame_id  + '/base_link'
        # how long to track the roomba
        self._time_to_track = task_request.time_to_track
        # overshoot from the center of the roomba
        self._x_overshoot = task_request.x_overshoot
        self._y_overshoot = task_request.y_overshoot

        if self._roomba_id is None:
            raise ValueError('A null roomba id was provided')

        # data about roombas
        self._roomba_odometry = None
        self._roomba_point = None
        # used for acceleration limiting
        self._current_velocity = None
        # keeping track of total track time
        self._start_time = None
        # transition from MCC
        self._transition = None
        # used by MCC to cancel task
        self._canceled = False
        # thread safe
        self._lock = threading.RLock()

        try:
            self._TRANSFORM_TIMEOUT = rospy.get_param('~transform_timeout')
            self._MAX_HORIZ_SPEED = rospy.get_param('~max_translation_speed')
            self._MAX_TASK_DIST = rospy.get_param('~max_roomba_dist')
            self._MAX_Z_VELOCITY = rospy.get_param('~max_z_velocity')
            x_pid_settings = PidSettings(rospy.get_param('~roomba_pid_settings/x_terms'))
            y_pid_settings = PidSettings(rospy.get_param('~roomba_pid_settings/y_terms'))
            TRACK_HEIGHT = rospy.get_param('~track_roomba_height')
        except KeyError as e:
            rospy.logerr('Could not lookup a parameter for track roomba task')
            raise

        self._z_holder = HeightHolder(TRACK_HEIGHT)
        self._height_checker = HeightSettingsChecker()
        self._limiter = AccelerationLimiter()
        self._x_pid = PidController(x_pid_settings)
        self._y_pid = PidController(y_pid_settings)

        if self._MAX_TASK_DIST < math.sqrt(self._x_overshoot**2
                            + self._y_overshoot**2):
            raise ValueError('The overshoot is outside the max distance')

        self._state = TrackRoombaTaskState.init

    def get_desired_command(self):
        with self._lock:
            if self._start_time is None:
                self._start_time = rospy.Time.now()
            
            if self._time_to_track != 0 and (rospy.Time.now() 
                - self._start_time >= rospy.Duration(self._time_to_track)):
                rospy.loginfo('TrackRoombaTask has tracked the roomba for the specified duration')
                return (TaskDone(),)

            if self._canceled:
                return (TaskCanceled(),)

            if (self._state == TrackRoombaTaskState.init):
                if (not self.topic_buffer.has_roomba_message()
                 or not self.topic_buffer.has_odometry_message()):
                    self._state = TrackRoombaTaskState.waiting
                else:
                    self._state = TrackRoombaTaskState.track

            if (self._state == TrackRoombaTaskState.waiting):
                if (not self.topic_buffer.has_roomba_message()
                 or not self.topic_buffer.has_odometry_message()):
                    return (TaskRunning(), NopCommand())
                else:
                    self._state = TrackRoombaTaskState.track

            if self._state == TrackRoombaTaskState.track:
                if not (self._height_checker.above_min_maneuver_height(
                        self.topic_buffer.get_odometry_message().pose.pose.position.z)):
                    return (TaskAborted(msg='TrackRoombaTask: Drone is too low to maneuver'),)
                elif not (self._z_holder.check_z_error(
                        self.topic_buffer.get_odometry_message().pose.pose.position.z)):
                    return (TaskAborted(msg='TrackRoombaTask: Z height error is too high'),)
                elif not self._check_roomba_in_sight():
                    return (TaskAborted(msg='TrackRoombaTask: The provided roomba is not in sight of quad'),)
                try:
                    roomba_transform = self.topic_buffer.get_tf_buffer().lookup_transform(
                                        'level_quad',
                                        self._roomba_id,
                                        rospy.Time(0),
                                        rospy.Duration(self._TRANSFORM_TIMEOUT))
                except (tf2_ros.LookupException,
                        tf2_ros.ConnectivityException,
                        tf2_ros.ExtrapolationException) as ex:
                    rospy.logerr('TrackRoombaTask: Exception when looking up transform')
                    rospy.logerr(ex.message)
                    return (TaskAborted(msg='Exception when looking up transform during TrackRoombaTask'),)

                # Create point centered at drone's center (0,0,0)
                stamped_point = PointStamped()
                stamped_point.point.x = 0
                stamped_point.point.y = 0
                stamped_point.point.z = 0

                # returns point distances of roomba to center point of level quad
                self._roomba_point = tf2_geometry_msgs.do_transform_point(
                                                    stamped_point, roomba_transform)

                if not self._check_max_roomba_range():
                    return (TaskAborted(msg='The provided roomba is not found or not close enough to the quad'),)

                roomba_x_velocity = self._roomba_odometry.twist.twist.linear.x
                roomba_y_velocity = self._roomba_odometry.twist.twist.linear.y
                roomba_velocity = math.sqrt(roomba_x_velocity**2 + roomba_y_velocity**2)

                # The overshoot is taking in the x velocity normalizing it and applying overshoot
                overshoot = math.sqrt(self._x_overshoot**2 + self._y_overshoot**2)
                x_overshoot = overshoot * math.cos(math.atan2(roomba_y_velocity, roomba_x_velocity)
                                                        + math.atan2(self._y_overshoot, self._x_overshoot))
                y_overshoot = overshoot * math.sin(math.atan2(roomba_y_velocity, roomba_x_velocity )
                                                        + math.atan2(self._y_overshoot, self._x_overshoot))

                x_diff = self._roomba_point.point.x + x_overshoot
                y_diff = self._roomba_point.point.y + y_overshoot

                x_success, x_response = self._x_pid.update(x_diff, rospy.Time.now(), False)
                y_success, y_response = self._y_pid.update(y_diff, rospy.Time.now(), False)

                # PID controller does setpoint - current; 
                # the difference from do_transform_point is from the drone to the roomba, 
                # which is the equivalent of current-setpoint, so take the negative response
                if x_success: 
                    x_vel_target = -x_response + roomba_x_velocity
                else: 
                    x_vel_target = roomba_x_velocity

                if y_success: 
                    y_vel_target = -y_response + roomba_y_velocity
                else: 
                    y_vel_target = roomba_y_velocity
                
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

                # capping acceleration
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
            
            return (TaskAborted(msg='Illegal state reached in Track Roomba task'),)

    # checks to see if passed in roomba id is in sight of quad
    def _check_roomba_in_sight(self):
        for odometry in self.topic_buffer.get_roomba_message().data:
            if odometry.child_frame_id == self._roomba_id:
                self._roomba_odometry = odometry
                return True
        return False

    # that the drone and roomba are both within a specified distance
    # in order to start/continue the task
    def _check_max_roomba_range(self):
        _distance_to_roomba = math.sqrt(self._roomba_point.point.x**2 + 
                            self._roomba_point.point.y**2)
        return (_distance_to_roomba <= self._MAX_TASK_DIST)

    def cancel(self):
        rospy.loginfo('TrackRoomba Task canceled')
        self._canceled = True
        return True
    
    def set_incoming_transition(self, transition):
        self._transition = transition

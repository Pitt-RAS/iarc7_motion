#!/usr/bin/env python

import math
import rospy
import tf2_ros
import tf2_geometry_msgs
import threading

from nav_msgs.msg import Odometry
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

class TrackRoombaTaskState(object):
    init = 0
    track = 1

class TrackRoombaTask(AbstractTask):

    def __init__(self, task_request):
        super(TrackRoombaTask, self).__init__()

        # id of the roomba we are tracking
        self._roomba_id = task_request.frame_id
        # how long to track the roomba
        self._time_to_track = task_request.time_to_track
        # overshoot from the center of the roomba
        self._x_overshoot = task_request.x_overshoot
        self._y_overshoot = task_request.y_overshoot

        # Used to calculate when to exit the task if a total
        # tracking time is specified
        self._task_start_time = None
        # Used to calculate the time spent locked on a roomba
        self._lock_start_time = None
        # transition from MCC
        self._transition = None
        # used by MCC to cancel task
        self._canceled = False
        # thread safe
        self._lock = threading.RLock()

        try:
            self._TRANSFORM_TIMEOUT = rospy.get_param('~transform_timeout')
            self._MAX_HORIZ_SPEED = rospy.get_param('~max_translation_speed')
            self._MAX_Z_VELOCITY = rospy.get_param('~max_z_velocity')
            self._MAX_ROOMBA_DIST = rospy.get_param('~max_roomba_dist')
            TRACK_HEIGHT = rospy.get_param('~track_roomba_height')
            X_PID_SETTINGS = PidSettings(rospy.get_param('~roomba_pid_settings/x_terms'))
            Y_PID_SETTINGS = PidSettings(rospy.get_param('~roomba_pid_settings/y_terms'))
            self._LOCK_DISTANCE = rospy.get_param('~track_completed_distance')
            self._LOCK_VELOCITY = rospy.get_param('~track_completed_vel_diff')
            self._LOCK_REQUIRED_DURATION  = rospy.Duration(rospy.get_param('~track_completed_time'))
        except KeyError as e:
            rospy.logerr('Could not lookup a parameter for track roomba task')
            raise

        self._z_holder = HeightHolder(TRACK_HEIGHT)
        self._height_checker = HeightSettingsChecker()
        self._x_pid = PidController(X_PID_SETTINGS)
        self._y_pid = PidController(Y_PID_SETTINGS)

        if self._MAX_ROOMBA_DIST < math.sqrt(self._x_overshoot**2
                            + self._y_overshoot**2):
            raise ValueError('The overshoot is outside the max distance')

        self._state = TrackRoombaTaskState.init

        self._linear_motion_profile_generator = \
                self.topic_buffer.get_linear_motion_profile_generator()

    def get_desired_command(self):
        with self._lock:
            if self._task_start_time is None:
                self._task_start_time = rospy.Time.now()

            if self._time_to_track != 0 and (rospy.Time.now()
                - self._task_start_time >= rospy.Duration(self._time_to_track)):
                rospy.loginfo('TrackRoombaTask has tracked the roomba for the specified duration')
                return (TaskDone(),)

            if self._canceled:
                return (TaskCanceled(),)

            if (self._state == TrackRoombaTaskState.init):
                if (not self.topic_buffer.has_roomba_message()
                 or not self.topic_buffer.has_odometry_message()):
                    return (TaskRunning(), NopCommand())
                else:
                    self._state = TrackRoombaTaskState.track

            if self._state == TrackRoombaTaskState.track:
                odometry = self.topic_buffer.get_odometry_message()

                # Check conditions that require aborting
                if not (self._height_checker.above_min_maneuver_height(
                        odometry.pose.pose.position.z)):
                    return (TaskAborted(msg='TrackRoombaTask: Drone is too low to maneuver'),)
                elif not (self._z_holder.check_z_error(
                          odometry.pose.pose.position.z)):
                    return (TaskAborted(msg='TrackRoombaTask: Z height error is too high'),)

                (roomba_odom_available, roomba_odometry) = \
                    self.topic_buffer.get_roomba_odometry(self._roomba_id)

                if not roomba_odom_available:
                    return (TaskAborted(msg='TrackRoombaTask: The tracked roombas odometry is not available'),)

                # Get the roomba's relative location
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
                roomba_point = tf2_geometry_msgs.do_transform_point(
                                                    stamped_point, roomba_transform)
                roomba_h_distance = math.sqrt(roomba_point.point.x**2 + roomba_point.point.y**2)

                if roomba_h_distance > self._MAX_ROOMBA_DIST:
                    return (TaskAborted(msg='The provided roomba is not found \
                                            or not close enough to the quad'),)

                odometry = self.topic_buffer.get_odometry_message()
                roomba_x_velocity = roomba_odometry.twist.twist.linear.x
                roomba_y_velocity = roomba_odometry.twist.twist.linear.y

                x_v_diff = odometry.twist.twist.linear.x - roomba_x_velocity
                y_v_diff = odometry.twist.twist.linear.y - roomba_y_velocity
                h_v_diff_mag = math.sqrt(x_v_diff**2 + y_v_diff**2)

                roomba_track_msg = Odometry()
                roomba_track_msg.header.stamp = rospy.Time.now()
                roomba_track_msg.pose.pose.position.x = roomba_point.point.x
                roomba_track_msg.pose.pose.position.y = roomba_point.point.y
                roomba_track_msg.pose.pose.position.z = roomba_h_distance
                roomba_track_msg.twist.twist.linear.x = x_v_diff
                roomba_track_msg.twist.twist.linear.y = y_v_diff
                roomba_track_msg.twist.twist.linear.z = h_v_diff_mag
                self.topic_buffer.publish_roomba_tracking_status(roomba_track_msg)

                # Evaluate whether or not the lock completion sequence was accomplished
                if self._time_to_track == 0.0 \
                   and roomba_h_distance <= self._LOCK_DISTANCE \
                   and h_v_diff_mag <= self._LOCK_VELOCITY:
                    if self._lock_start_time is not None:
                        if rospy.Time.now() - self._lock_start_time > self._LOCK_REQUIRED_DURATION:
                            rospy.loginfo('TrackRoombaTask has locked on the roomba for the required amount of time')

                            task_messages = self.topic_buffer.get_task_message_dictionary()
                            task_messages['track_x_i_accumulator'] = self._x_pid.get_accumulator()
                            task_messages['track_y_I_accumulator'] = self._y_pid.get_accumulator()

                            return (TaskDone(),)
                    else:
                        self._lock_start_time = rospy.Time.now()
                else:
                    self._lock_start_time = None

                # Calculate the overshoot distance in the drones frame
                overshoot = math.sqrt(self._x_overshoot**2 + self._y_overshoot**2)
                x_overshoot = overshoot * math.cos(math.atan2(roomba_y_velocity, roomba_x_velocity)
                                                        + math.atan2(self._y_overshoot, self._x_overshoot))
                y_overshoot = overshoot * math.sin(math.atan2(roomba_y_velocity, roomba_x_velocity )
                                                        + math.atan2(self._y_overshoot, self._x_overshoot))

                # Find distance between the drones position and the desired position
                x_p_diff = roomba_point.point.x + x_overshoot
                y_p_diff = roomba_point.point.y + y_overshoot

                x_success, x_response = self._x_pid.update(x_p_diff, rospy.Time.now(), False)
                y_success, y_response = self._y_pid.update(y_p_diff, rospy.Time.now(), False)

                # PID controller does setpoint - current;
                # the difference from do_transform_point is from the drone to the roomba,
                # which is the equivalent of current-setpoint, so take the negative response
                # This is done so that  the drones position in the map frame doesn't
                # have to be calculated
                if x_success:
                    x_vel_target = -x_response + roomba_x_velocity
                else:
                    x_vel_target = roomba_x_velocity

                if y_success:
                    y_vel_target = -y_response + roomba_y_velocity
                else:
                    y_vel_target = roomba_y_velocity

                # Get the z response
                predicted_motion_point = \
                    self._linear_motion_profile_generator.expected_point_at_time(
                        rospy.Time.now())

                current_height = odometry.pose.pose.position.z
                predicted_height = \
                    predicted_motion_point.motion_point.pose.position.z

                z_vel_target, z_reset = \
                    self._z_holder.get_height_hold_response(
                        current_height,
                        predicted_height)

                # Cap the horizontal velocity
                h_vel_target = math.sqrt(x_vel_target**2 + y_vel_target**2)
                if h_vel_target > self._MAX_HORIZ_SPEED:
                    x_vel_target = x_vel_target * (self._MAX_HORIZ_SPEED/h_vel_target)
                    y_vel_target = y_vel_target * (self._MAX_HORIZ_SPEED/h_vel_target)

                # Cap the z velocity target
                if (abs(z_vel_target) > self._MAX_Z_VELOCITY):
                    z_vel_target =  math.copysign(self._MAX_Z_VELOCITY, z_vel_target)

                desired_vel = [x_vel_target, y_vel_target, z_vel_target]

                velocity = TwistStamped()
                velocity.header.frame_id = 'level_quad'
                velocity.header.stamp = rospy.Time.now()
                velocity.twist.linear.x = desired_vel[0]
                velocity.twist.linear.y = desired_vel[1]
                velocity.twist.linear.z = desired_vel[2]

                if z_reset:
                    velocity_command = VelocityCommand(velocity,
                                        start_position_z=odometry.pose.pose.position.z)
                else:
                    velocity_command = VelocityCommand(velocity)

                return (TaskRunning(), velocity_command)

            return (TaskAborted(msg='Illegal state reached in Track Roomba task'),)

    def cancel(self):
        with self._lock:
            rospy.loginfo('TrackRoomba Task canceled')
            self._canceled = True
            return True

    def set_incoming_transition(self, transition):
        with self._lock:
            self._transition = transition

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

class HitRoombaTaskState(object):
    init = 0
    waiting = 1
    descent = 2
    ascent = 3
    failure = 4

class HitDetectionState(object):
    disarmed = 0
    armed = 1
    hit_detected = 2

class HitRoombaTask(AbstractTask):

    def __init__(self, task_request):
        super(HitRoombaTask, self).__init__()

        # id of roomba to hit
        self._roomba_id = task_request.frame_id

        if self._roomba_id == '':
            raise ValueError('An invalid roomba id was provided to HitRoombaTask')

        # data about roombas
        self._distance_to_roomba = None
        # task was canceled by MCC
        self._canceled = False
        # transition from MCC
        self._transition = None
        # thread safe
        self._lock = threading.RLock()

        try:
            self._TRANSFORM_TIMEOUT = rospy.get_param('~transform_timeout')
            self._MAX_HORIZ_SPEED = rospy.get_param('~max_translation_speed')
            self._MAX_Z_VELOCITY = rospy.get_param('~max_z_velocity')
            self._DESCENT_VELOCITY = rospy.get_param('~hit_descent_velocity')
            self._SAFE_ASCENT_VELOCITY = rospy.get_param('~hit_safe_ascent_velocity')
            self._ASCENT_ACCELERATION = rospy.get_param('~hit_ascent_acceleration')
            self._MAX_ROOMBA_DESCENT_DIST = rospy.get_param('~max_roomba_descent_dist')
            self._ROOMBA_HIT_ARM_THRESHOLD = rospy.get_param('~roomba_hit_arm_threshold')
            self._ROOMBA_HIT_DETECTED_THRESHOLD = rospy.get_param('~roomba_hit_detected_threshold')
            self._RECOVERY_HEIGHT = rospy.get_param('~track_roomba_height')
            x_pid_settings = PidSettings(rospy.get_param('~hit_roomba_pid_settings/x_terms'))
            y_pid_settings = PidSettings(rospy.get_param('~hit_roomba_pid_settings/y_terms'))
        except KeyError as e:
            rospy.logerr('Could not lookup a parameter for hit roomba task')
            raise

        try:
            task_messages = self.topic_buffer.get_task_message_dictionary()
            x_pid_settings.i_accumulator_initial_value = task_messages['track_x_i_accumulator']
            y_pid_settings.i_accumulator_initial_value = task_messages['track_y_i_accumulator']
        except KeyError as e:
            x_pid_settings.i_accumulator_initial_value = None
            y_pid_settings.i_accumulator_initial_value = None
            rospy.logwarn('Hit Roomba Task could not get track roombas accumulator values')
        finally:
            task_messages['track_x_i_accumulator'] = None
            task_messages['track_y_i_accumulator'] = None

        self._x_pid = PidController(x_pid_settings)
        self._y_pid = PidController(y_pid_settings)

        self._ascension_begin_deceleration_height = None
        self._ascent_velocity = None

        self._hit_detection_state = HitDetectionState.disarmed
        self._state = HitRoombaTaskState.init

    def get_desired_command(self):
        with self._lock:
            if self._canceled:
                return (TaskCanceled(),)

            if (self._state == HitRoombaTaskState.init):
                if (not self.topic_buffer.has_roomba_message()
                 or not self.topic_buffer.has_odometry_message()
                 or not self.topic_buffer.has_landing_message()):
                    return (TaskRunning(), NopCommand())
                else:
                    self._hit_detection_state = HitDetectionState.disarmed
                    self._state = HitRoombaTaskState.descent

            if (self._state == HitRoombaTaskState.descent):

                # Check and see if the roomba's odometry is available
                (roomba_odom_available, roomba_odometry) = \
                    self.topic_buffer.get_roomba_odometry(self._roomba_id)
                if not roomba_odom_available:
                    return (TaskAborted(msg='The provided roomba is not in sight of quad'),)

                odometry = self.topic_buffer.get_odometry_message()

                # If we have landed transition to ascension
                if self.topic_buffer.get_landing_message().data:
                    return self._transition_to_ascension()

                # If the hit detector has gone off transition to ascension
                if self._hit_detection_state == HitDetectionState.disarmed:
                    if odometry.twist.twist.linear.z < self._ROOMBA_HIT_ARM_THRESHOLD:
                        self._hit_detection_state = HitDetectionState.armed
                elif self._hit_detection_state == HitDetectionState.armed:
                    if odometry.twist.twist.linear.z > self._ROOMBA_HIT_DETECTED_THRESHOLD:
                        self._hit_detection_state == HitDetectionState.hit_detected
                        return self._transition_to_ascension()

                # Get the roomba's relative distance to the drone
                try:
                    roomba_transform = self.topic_buffer.get_tf_buffer().lookup_transform(
                                        'level_quad',
                                        self._roomba_id,
                                        rospy.Time(0),
                                        rospy.Duration(self._TRANSFORM_TIMEOUT))
                except (tf2_ros.LookupException,
                        tf2_ros.ConnectivityException,
                        tf2_ros.ExtrapolationException) as ex:
                    rospy.logerr('HitRoombaTask: Exception when looking up transform')
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
                x_p_diff = self._roomba_point.point.x
                y_p_diff = self._roomba_point.point.y
                roomba_h_distance = math.sqrt(x_p_diff**2 + y_p_diff**2)

                roomba_x_velocity = roomba_odometry.twist.twist.linear.x
                roomba_y_velocity = roomba_odometry.twist.twist.linear.y

                x_v_diff = odometry.twist.twist.linear.x - roomba_x_velocity
                y_v_diff = odometry.twist.twist.linear.y - roomba_y_velocity
                h_v_diff_mag = math.sqrt(x_v_diff**2 + y_v_diff**2)

                # Publish roomba tracking debugging information
                roomba_track_msg = Odometry()
                roomba_track_msg.header.stamp = rospy.Time.now()
                roomba_track_msg.pose.pose.position.x = x_p_diff
                roomba_track_msg.pose.pose.position.y = y_p_diff
                roomba_track_msg.pose.pose.position.x = roomba_h_distance
                roomba_track_msg.twist.twist.linear.x = x_v_diff
                roomba_track_msg.twist.twist.linear.y = y_v_diff
                roomba_track_msg.twist.twist.linear.z = h_v_diff_mag
                self.topic_buffer.publish_roomba_tracking_status(roomba_track_msg)

                # Make sure that the drone is close enough to the roomba
                if roomba_h_distance <= self._MAX_ROOMBA_DESCENT_DIST:
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

                    h_vel_target = math.sqrt(x_vel_target**2 + y_vel_target**2)

                    if h_vel_target > self._MAX_HORIZ_SPEED:
                        x_vel_target = x_vel_target * (self._MAX_HORIZ_SPEED/h_vel_target)
                        y_vel_target = y_vel_target * (self._MAX_HORIZ_SPEED/h_vel_target)

                    z_vel_target = self._DESCENT_VELOCITY
                    if (abs(z_vel_target) > self._MAX_Z_VELOCITY):
                        z_vel_target = math.copysign(self._MAX_Z_VELOCITY, z_vel_target)
                        rospy.logerr('Hit roomba task descent velocity higher than global max velocity')

                    desired_vel = [x_vel_target, y_vel_target, z_vel_target]

                    velocity = TwistStamped()
                    velocity.header.frame_id = 'level_quad'
                    velocity.header.stamp = rospy.Time.now()
                    velocity.twist.linear.x = desired_vel[0]
                    velocity.twist.linear.y = desired_vel[1]
                    velocity.twist.linear.z = desired_vel[2]

                    return (TaskRunning(), VelocityCommand(velocity))
                else:
                    self._state = HitRoombaTaskState.failure
                    rospy.logerr('Hit Roomba Task aborted descent due to distance from roomba')

            # Ascend very fast
            if (self._state == HitRoombaTaskState.ascent):
                odometry = self.topic_buffer.get_odometry_message()

                if odometry.pose.pose.position.z > self._ascension_begin_deceleration_height:
                    return (TaskDone(), VelocityCommand(acceleration=self._ASCENT_ACCELERATION))
                else:
                    velocity = TwistStamped()
                    velocity.header.frame_id = 'level_quad'
                    velocity.header.stamp = rospy.Time.now()
                    velocity.twist.linear.z = self._ascent_velocity
                    return (TaskRunning(),
                            VelocityCommand(velocity, acceleration=self._ASCENT_ACCELERATION))

            # Ascend at a safe speed
            if (self._state == HitRoombaTaskState.failure):
                odometry = self.topic_buffer.get_odometry_message()

                if odometry.pose.pose.position.z > self._RECOVERY_HEIGHT:
                    return (TaskDone(), VelocityCommand())
                else:
                    velocity = TwistStamped()
                    velocity.header.frame_id = 'level_quad'
                    velocity.header.stamp = rospy.Time.now()
                    velocity.twist.linear.z = self._SAFE_ASCENT_VELOCITY
                    return (TaskAborted(), VelocityCommand(velocity))

            return (TaskAborted(msg='Illegal state reached in Hit Roomba Task' ),)

    def _transition_to_ascension(self):
        odometry = self.topic_buffer.get_odometry_message()

        self._ascension_begin_deceleration_height = \
            (self._RECOVERY_HEIGHT - odometry.pose.pose.position.z) / 2.0

        ascent_acceleration_time = \
            math.sqrt(2.0
                      * self._ascension_begin_deceleration_height
                      / self._ASCENT_ACCELERATION)

        self._ascent_velocity = \
            ascent_acceleration_time * self._ASCENT_ACCELERATION

        if (abs(self._ascent_velocity) > self._MAX_Z_VELOCITY):
            self._ascent_velocity = math.copysign(self._MAX_Z_VELOCITY, z_vel_target)
            rospy.logerr('Hit roomba task calculated ascent velocity higher than global max velocity')

        velocity = TwistStamped()
        velocity.header.frame_id = 'level_quad'
        velocity.header.stamp = rospy.Time.now()
        velocity.twist.linear.z = self._ascent_velocity

        self._state = HitRoombaTaskState.ascent
        return (TaskRunning(), VelocityCommand(velocity,
                                               start_position_z
                                                   = odometry.pose.pose.position.z,
                                               start_velocity_z
                                                   = 0.0,
                                               acceleration=self._ASCENT_ACCELERATION))

    def cancel(self):
        with self._lock:
            rospy.loginfo('HitRoomba Task canceled')
            self._canceled = True
            return True

    def set_incoming_transition(self, transition):
        with self._lock:
            self._transition = transition

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
from task_utilities.acceleration_limiter import AccelerationLimiter

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
        self._roomba_odometry = None
        self._roomba_point = None
        self._distance_to_roomba = None
        # used to limit accelerations
        self._limiter = AccelerationLimiter()
        # task was canceled by MCC
        self._canceled = False
        # used in accleration limitting
        self._current_velocity = None
        # transition from MCC
        self._transition = None
        # thread safe
        self._lock = threading.RLock()

        try:
            self._TRANSFORM_TIMEOUT = rospy.get_param('~transform_timeout')
            self._MAX_HORIZ_SPEED = rospy.get_param('~max_translation_speed')
            self._MAX_START_TASK_DIST = rospy.get_param('~hit_roomba_max_start_dist')
            self._MAX_Z_VELOCITY = rospy.get_param('~max_z_velocity')
            self._descent_velocity = rospy.get_param('~hit_descent_velocity')
            self._ascent_velocity = rospy.get_param('~hit_ascent_velocity')
            self._ascent_acceleration = rospy.get_param('~hit_ascent_acceleration')
            self._max_roomba_descent_dist = rospy.get_param('~max_roomba_descent_dist')
            self._roomba_hit_arm_threshold = rospy.get_param('~roomba_hit_arm_threshold')
            self._roomba_hit_detected_threshold = rospy.get_param('~roomba_hit_detected_threshold')
            self._recovery_height = rospy.get_param('~track_roomba_height')
            x_pid_settings = PidSettings(rospy.get_param('~roomba_pid_settings/x_terms'))
            y_pid_settings = PidSettings(rospy.get_param('~roomba_pid_settings/y_terms'))
        except KeyError as e:
            rospy.logerr('Could not lookup a parameter for hit roomba task')
            raise

        self._x_pid = PidController(x_pid_settings)
        self._y_pid = PidController(y_pid_settings)

        self._begin_ascension_deceleration_height = None

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
                    self._state = HitRoombaTaskState.waiting
                else:
                    self._state = HitRoombaTaskState.descent

            if (self._state == HitRoombaTaskState.waiting):
                if (not self.topic_buffer.has_roomba_message()
                 or not self.topic_buffer.has_odometry_message()
                 or not self.topic_buffer.has_landing_message()):
                    return (TaskRunning(), NopCommand())
                else:
                    self._hit_detection_state = HitDetectionState.disarmed
                    self._state = HitRoombaTaskState.descent

            if (self._state == HitRoombaTaskState.descent):
                rospy.logwarn('Descending')
                if not self._check_roomba_in_sight():
                    return (TaskAborted(msg='The provided roomba is not in sight of quad'),)

                odometry = self.topic_buffer.get_odometry_message()

                if self._on_ground():
                    return self._transition_to_ascension()

                if self._hit_detection_state == HitDetectionState.disarmed:
                    if odometry.twist.twist.linear.z < self._roomba_hit_arm_threshold:
                        self._hit_detection_state = HitDetectionState.armed
                elif self._hit_detection_state == HitDetectionState.armed:
                    if odometry.twist.twist.linear.z > self._roomba_hit_detected_threshold:
                        self._hit_detection_state == HitDetectionState.hit_detected
                        return self._transition_to_ascension()

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

                # uses _roomba_point to determine if quad too far from roomba
                if not self._check_max_roomba_range():
                    return (TaskAborted(msg='The provided roomba is not close enough to the quad'),)

                roomba_x_velocity = self._roomba_odometry.twist.twist.linear.x
                roomba_y_velocity = self._roomba_odometry.twist.twist.linear.y

                x_diff = self._roomba_point.point.x
                y_diff = self._roomba_point.point.y

                x_success, x_response = self._x_pid.update(x_diff, rospy.Time.now(), False)
                y_success, y_response = self._y_pid.update(y_diff, rospy.Time.now(), False)

                # PID controller does setpoint - current;
                # the difference from do_transform_point is from the drone to the roomba,
                # which is the equivalent of current-setpoint, so take the negative response
                if x_success:
                    x_vel_target = -x_response + roomba_x_velocity
                #else:
                #    x_vel_target = roomba_x_velocity

                if y_success:
                    y_vel_target = -y_response + roomba_y_velocity
                #else:
                #    y_vel_target = roomba_y_velocity

                # make sure we are close enough before we descend
                if self._distance_to_roomba <= self._max_roomba_descent_dist:
                    z_vel_target = self._descent_velocity
                    desired_vel = [roomba_x_velocity, roomba_y_velocity, z_vel_target]

                    velocity = TwistStamped()
                    velocity.header.frame_id = 'level_quad'
                    velocity.header.stamp = rospy.Time.now()
                    velocity.twist.linear.x = desired_vel[0]
                    velocity.twist.linear.y = desired_vel[1]
                    velocity.twist.linear.z = desired_vel[2]

                    self._current_velocity = desired_vel

                    return (TaskRunning(), VelocityCommand(velocity))

                else:
                    self._state = HitRoombaTaskState.failure
                    rospy.logwarn('hit roomba task not close enough to roomba to descend')
                    # cap velocity
                    # vel_target = math.sqrt(x_vel_target**2 + y_vel_target**2)

                    # if vel_target > self._MAX_HORIZ_SPEED:
                    #     x_vel_target = x_vel_target * (self._MAX_HORIZ_SPEED/vel_target)
                    #     y_vel_target = y_vel_target * (self._MAX_HORIZ_SPEED/vel_target)

                    # if (abs(z_vel_target) > self._MAX_Z_VELOCITY):
                    #     z_vel_target = math.copysign(self._MAX_Z_VELOCITY, z_vel_target)

                    # desired_vel = [x_vel_target, y_vel_target, z_vel_target]

                    # drone_vel_x = odometry.twist.twist.linear.x
                    # drone_vel_y = odometry.twist.twist.linear.y
                    # drone_vel_z = odometry.twist.twist.linear.z

                    # if self._current_velocity is None:
                    #     self._current_velocity = [drone_vel_x, drone_vel_y, drone_vel_z]

                    # desired_vel = self._limiter.limit_acceleration(self._current_velocity, desired_vel)

            if (self._state == HitRoombaTaskState.ascent):
                odometry = self.topic_buffer.get_odometry_message()

                if odometry.pose.pose.position.z > self._begin_ascension_deceleration_height:
                    return (TaskDone(), VelocityCommand())
                else:
                    velocity = TwistStamped()
                    velocity.header.frame_id = 'level_quad'
                    velocity.header.stamp = rospy.Time.now()
                    velocity.twist.linear.z = self._ascent_velocity
                    return (TaskRunning(), VelocityCommand(velocity,
                                                           acceleration=self._ascent_acceleration))

            if (self._state == HitRoombaTaskState.failure):
                odometry = self.topic_buffer.get_odometry_message()

                if odometry.pose.pose.position.z > self._recovery_height:
                    return (TaskDone(), VelocityCommand())
                else:
                    velocity = TwistStamped()
                    velocity.header.frame_id = 'level_quad'
                    velocity.header.stamp = rospy.Time.now()
                    velocity.twist.linear.z = self._ascent_velocity/2.0
                    return (TaskRunning(), VelocityCommand(velocity))

            return (TaskAborted(msg='Illegal state reached in Hit Roomba Task' ),)

    def _transition_to_ascension(self):
        odometry = self.topic_buffer.get_odometry_message()

        self._begin_ascension_deceleration_height \
            = (self._recovery_height - odometry.pose.pose.position.z) / 2.0

        velocity = TwistStamped()
        velocity.header.frame_id = 'level_quad'
        velocity.header.stamp = rospy.Time.now()
        velocity.twist.linear.z = self._ascent_velocity

        self._state = HitRoombaTaskState.ascent
        return (TaskRunning(), VelocityCommand(velocity,
                                               start_position_z
                                                   = odometry.pose.pose.position.z,
                                               start_velocity_z
                                                   = odometry.twist.twist.linear.z,
                                               acceleration=self._ascent_acceleration))

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
        self._distance_to_roomba = math.sqrt(self._roomba_point.point.x**2 +
                            self._roomba_point.point.y**2)

        return (self._distance_to_roomba <= self._MAX_START_TASK_DIST)

    def cancel(self):
        rospy.loginfo('HitRoomba Task canceled')
        self._canceled = True
        return True

    def _on_ground(self):
        return self.topic_buffer.get_landing_message().data

    def set_incoming_transition(self, transition):
        self._transition = transition

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
                                      NopCommand)

from task_utilities.height_settings_checker import HeightSettingsChecker

class HoldPositionTaskStates:
    init = 0
    waiting = 1
    holding = 2

class HoldPositionTask(AbstractTask):

    def __init__(self, task_request):
        self._hold_current_position = task_request.hold_current_position

        self._drone_odometry = None
        self._canceled = False
        self._transition = None

        self._lock = threading.RLock()
        self._height_checker = HeightSettingsChecker()

        if not self._hold_current_position:
            self._x_position = task_request.x_position
            self._y_position = task_request.y_position
            self._z_position = task_request.z_position

            if not self._height_checker.above_min_maneuver_height(self._z_position):
                raise ValueError('An invalid position was provided')
        else: 
            self._x_position = None
            self._y_position = None
            self._z_position = None

        self._current_velocity_sub = rospy.Subscriber(
            '/odometry/filtered', Odometry,
            self._current_velocity_callback)

        try:
            self._MAX_RANGE = rospy.get_param('~max_holding_range')
            self._MAX_TRANSLATION_SPEED = rospy.get_param('~max_translation_speed')
            self._MAX_Z_VELOCITY = rospy.get_param('~max_z_velocity')
            self._K_X = rospy.get_param('~k_position_z')
            self._K_Y = rospy.get_param('~k_position_z')
            self._K_Z = rospy.get_param('~k_position_z')
        except KeyError as e:
            rospy.logerr('Could not lookup a parameter for hold position task')
            raise

        self._state = HoldPositionTaskStates.init

    def _current_velocity_callback(self, data):
        with self._lock:
            self._drone_odometry = data

    def get_desired_command(self):
        with self._lock:
            if (self._state == HoldPositionTaskStates.init
                    or self._state == HoldPositionTaskStates.waiting):
                if self._drone_odometry is None:
                    self._state = HoldPositionTaskStates.waiting
                    return (TaskRunning(), NopCommand())
                
                self._set_targets()
                self._state = HoldPositionTaskStates.holding
                
                if not self._check_max_error():
                    return (TaskAborted(msg='Desired position is too far away, task not for translation.'),)

                return (TaskRunning(), NopCommand())

            elif (self._state == HoldPositionTaskStates.holding):
                if not (self._height_checker.above_min_maneuver_height(
                            self._drone_odometry.pose.pose.position.z)):
                    return (TaskAborted(msg='Drone is too low'),)

                if not (self._check_max_error()):
                    return (TaskAborted(msg='Error from desired point is too great'),)

                if self._canceled:
                    return (TaskCanceled(),)

                # p-controller
                x_vel_target = ((self._x_position - self._drone_odometry.pose.pose.position.x)
                                    * self._K_X)
                y_vel_target = ((self._y_position - self._drone_odometry.pose.pose.position.y) 
                                    * self._K_Y)
                z_vel_target = ((self._z_position - self._drone_odometry.pose.pose.position.z) 
                                    * self._K_Z)

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

            else:
                return (TaskAborted(msg='Illegal State Reached'),)
            
    def cancel(self):
        rospy.loginfo('HoldPosition Task canceled')
        self._canceled = True

    def _check_max_error(self):
        x_vel_target = (self._x_position - self._drone_odometry.pose.pose.position.x)
        y_vel_target = (self._y_position - self._drone_odometry.pose.pose.position.y)
        z_vel_target = (self._z_position - self._drone_odometry.pose.pose.position.z)

        _distance_to_point = math.sqrt(x_vel_target**2 + y_vel_target**2 + z_vel_target**2)
        
        return (_distance_to_point <= self._MAX_RANGE)

    def _set_targets(self):
        if (self._x_position is None and self._y_position is None
                                     and self._z_position is None):
            self._x_position = self._drone_odometry.pose.pose.position.x
            self._y_position = self._drone_odometry.pose.pose.position.y
            self._z_position = self._drone_odometry.pose.pose.position.z
    
    def send_transition(self, transition):
        self._transition = transition

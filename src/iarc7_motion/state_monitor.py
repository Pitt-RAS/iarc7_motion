#!/usr/bin/env python
import actionlib
import rospy
import sys
import threading
import traceback

from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from iarc7_msgs.msg import TwistStampedArray, OdometryArray
from iarc7_msgs.msg import FlightControllerStatus

from iarc_tasks.takeoff_task import TakeoffTask
from iarc_tasks.land_task import LandTask
from iarc_tasks.test_task import TestTask
from iarc_tasks.xyztranslation_task import XYZTranslationTask
from iarc_tasks.track_roomba_task import TrackRoombaTask
from iarc_tasks.hit_roomba_task import HitRoombaTask
from iarc_tasks.block_roomba_task import BlockRoombaTask
from iarc_tasks.hold_position_task import HoldPositionTask
from iarc_tasks.height_recovery_task import HeightRecoveryTask
from iarc_tasks.velocity_task import VelocityTask


class StateMonitor:

    def __init__(self):
        # state data
        self._roombas = None
        self._drone_odometry = None
        self._arm_status = None
        
        # current state of HLM
        self._initialized = False
        self._waiting_on_takeoff = True
        self._waiting_on_recovery = False

        # to keep things thread safe
        self._lock = threading.RLock()
        
        # info needed to do sanity checking and state monitoring
        self._roomba_status_sub = rospy.Subscriber(
            'roombas', OdometryArray, 
            self._receive_roomba_status)

        self._current_velocity_sub = rospy.Subscriber(
            '/odometry/filtered', Odometry,
            self._recieve_drone_odometry)

        self._drone_arm_status = rospy.Subscriber(
            '/fc_status', FlightControllerStatus, 
            self._receive_arm_status)

        try:
            # task timeout time to decelerate
            self._task_timeout_deceleration_time = rospy.Duration(
                rospy.get_param('~task_timeout_deceleration_time'))
            # minimum safe height to manuever at
            self._MIN_MANEUVER_HEIGHT = rospy.get_param('~min_maneuver_height')

        except KeyError as e:
            rospy.logerr('Could not lookup a parameter for State Monitor')
            raise

    # checks task transitions before executing it
    def check_transition(self, task):
        with self._lock: 
            passed = False

            if self._waiting_on_takeoff: 
                passed = isinstance(task, TakeoffTask)
            elif isinstance(task, LandTask):
                self._waiting_on_takeoff = True
                passed = True
            elif self._waiting_on_recovery:
                passed = isinstance(task, HeightRecoveryTask)
            elif self._below_min_manuever_height():
                passed = isinstance(task, TakeoffTask) or isinstance(task, HeightRecoveryTask)
            else: 
                passed = True

            if isinstance(task, TakeoffTask):
                self._waiting_on_takeoff = False
            elif isinstance(task, BlockRoombaTask) or isinstance(task, HitRoombaTask):
                self._waiting_on_recovery = True
            elif isinstance(task, HeightRecoveryTask):
                self._waiting_on_recovery = False

            return passed

    # checks to see if drone above min manuever height
    def _below_min_manuever_height(self):
        with self._lock:
            if self._drone_odometry is None:
                return True
            else:
                return (self._drone_odometry.pose.pose.position.z 
                            < self._MIN_MANEUVER_HEIGHT)

    # fills out the rest of Intermediary State for the task
    def fill_out_transition(self, state):
        while ((self._drone_odometry is None or self._roombas is None 
                or self._arm_status is None) and not rospy.is_shutdown()):
            pass
        with self._lock:
            state.drone_odometry = self._drone_odometry
            state.roombas = self._roombas
            state.arm_status = self._arm_status
            return state
    
    # Handles no task running timeouts
    def get_timeout(self, last_twist):
        with self._lock:
            commands = TwistStampedArray()

            if last_twist is None:
                last_twist = TwistStamped()
                last_twist.header.stamp = rospy.Time.now()

            # send future twist with zero velocity
            # so that LLM will interpolate between the 
            # current velocity and 0m/s to avoid impulse in velocity
            future_twist = TwistStamped()
            future_twist.header.stamp = (rospy.Time.now() + 
                self._task_timeout_deceleration_time)

            commands.twists = [last_twist, future_twist]
            return commands

    """
    Callbacks for publishers
    
    Types: 
        drone odometry: odometry (position, velocity, etc.) of drone
        roomba status: odometry (position, velocity, etc.) of
            all roombas in sight of drone
    """
    def _recieve_drone_odometry(self, data):
        with self._lock:
            self._drone_odometry = data

    def _receive_roomba_status(self, data):
        with self._lock:
            self._roombas = data

    def _receive_arm_status(self, data):
        with self._lock:
            self._arm_status = data

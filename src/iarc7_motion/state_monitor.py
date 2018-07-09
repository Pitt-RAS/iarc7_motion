#!/usr/bin/env python
import actionlib
from enum import Enum
import rospy
import sys
import threading
import traceback

from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from iarc7_msgs.msg import TwistStampedArray, OdometryArray, ObstacleArray
from iarc7_msgs.msg import FlightControllerStatus

from iarc7_safety.iarc_safety_exception import (IARCSafetyException,
                                                IARCFatalSafetyException)

import iarc_tasks.task_states as task_states

from iarc_tasks.takeoff_task import TakeoffTask
from iarc_tasks.land_task import LandTask
from iarc_tasks.test_task import TestTask
from iarc_tasks.xyztranslation_task import XYZTranslationTask
from iarc_tasks.track_roomba_task import TrackRoombaTask
from iarc_tasks.go_to_roomba_task import GoToRoombaTask
from iarc_tasks.hit_roomba_task import HitRoombaTask
from iarc_tasks.block_roomba_task import BlockRoombaTask
from iarc_tasks.hold_position_task import HoldPositionTask
from iarc_tasks.height_recovery_task import HeightRecoveryTask
from iarc_tasks.velocity_task import VelocityTask
from iarc_tasks.test_planner_task import TestPlannerTask

class RobotStates(Enum):
    WAITING_ON_TAKEOFF = 1
    TAKEOFF_FAILED = 2
    LANDING_FAILED = 3
    WAITING_ON_RECOVERY = 4
    RECOVERY_FAILED = 5
    NORMAL = 6
    FATAL = 7
    SAFETY_ACTIVE = 8

class StateMonitor:

    def __init__(self):
        # state data
        self._roombas = None
        self._obstacles = None
        self._drone_odometry = None
        self._arm_status = None

        # current state of the drone
        self._state = RobotStates.WAITING_ON_TAKEOFF
        self._BELOW_MIN_MAN_HEIGHT = True

        # last task and its end state
        self._last_task = None
        self._last_task_end_state = None

        # to keep things thread safe
        self._lock = threading.RLock()

        # info needed to do sanity checking and state monitoring
        self._roomba_status_sub = rospy.Subscriber(
            'roombas', OdometryArray,
            self._receive_roomba_status)

        self._obstacle_sub = rospy.Subscriber(
            '/obstacles', ObstacleArray,
            self._receive_obstacle_status)

        self._current_velocity_sub = rospy.Subscriber(
            '/odometry/filtered', Odometry,
            self._receive_drone_odometry)

        self._drone_arm_status = rospy.Subscriber(
            '/fc_status', FlightControllerStatus,
            self._receive_arm_status)

        try:
            # minimum safe height to manuever at
            self._MIN_MANEUVER_HEIGHT = rospy.get_param('~min_maneuver_height')

        except KeyError as e:
            rospy.logerr('Could not lookup a parameter for State Monitor')
            raise

    # checks task transitions before executing it
    def check_transition(self, task):
        with self._lock:
            passed = True
            if self._state == RobotStates.SAFETY_ACTIVE:
                if not isinstance(task, LandTask):
                    raise IARCFatalSafetyException(
                        'StateMonitor has been told we are safety active and landing was not requested')
            elif self._state == RobotStates.WAITING_ON_TAKEOFF:
                if not isinstance(task, TakeoffTask):
                    passed = False
            elif (self._state == RobotStates.TAKEOFF_FAILED or
                self._state == RobotStates.LANDING_FAILED
                or self._state == RobotStates.RECOVERY_FAILED):
                raise IARCSafetyException('A critical task failed')
            elif self._state == RobotStates.WAITING_ON_RECOVERY:
                if not isinstance(task, HeightRecoveryTask):
                    passed = False
            elif self._state == RobotStates.NORMAL:
                pass
            elif self._state == RobotStates.FATAL:
                raise IARCFatalSafetyException('StateMonitor determined robot is in a fatal state')
            else:
                raise IARCFatalSafetyException('StateMonitor is in an invalid state')

            if passed:
                self._last_task = task

            return passed

    # public function to receive last task's ending state
    # and transitions the state of the robot
    def set_last_task_end_state(self, state):
        with self._lock:
            self._last_task_end_state = state

            if self._state == RobotStates.SAFETY_ACTIVE:
                pass

            # If the tasked completed succesfully
            elif isinstance(state, task_states.TaskDone):
                if isinstance(self._last_task, LandTask):
                    self._state = RobotStates.WAITING_ON_TAKEOFF
                elif (isinstance(self._last_task, BlockRoombaTask)):
                    self._state = RobotStates.WAITING_ON_RECOVERY
                else:
                    self._state = RobotStates.NORMAL

            # If the task was canceled, there might be cleanup actions
            # to perform
            elif isinstance(state, task_states.TaskCanceled):
                # assumes that canceling land results in velocity mode
                if isinstance(self._last_task, LandTask): 
                    self._state = RobotStates.NORMAL
                # Takeoff needs to take the drone above the safe height
                # Hit roomba if canceled might not have taken the drone back up
                elif (isinstance(self._last_task, TakeoffTask)
                     or isinstance(self._last_task, HitRoombaTask)):
                    if not self._BELOW_MIN_MAN_HEIGHT:
                        self._state = RobotStates.NORMAL
                    else:
                        rospy.logerr('Takeoff did not finish when it was canceled')
                        self._state = RobotStates.FATAL
                # Block roomba can never bring the drone back up
                elif isinstance(self._last_task, BlockRoombaTask):
                    self._state = RobotStates.WAITING_ON_RECOVERY
                else:
                    self._state = RobotStates.NORMAL

            # If the task aborted there is probably a situation that
            # needs to be responded to
            elif (isinstance(state, task_states.TaskAborted)
                or isinstance(state, task_states.TaskFailed)):
                if isinstance(self._last_task, TakeoffTask):
                    self._state = RobotStates.TAKEOFF_FAILED
                elif isinstance(self._last_task, LandTask):
                    self._state = RobotStates.LANDING_FAILED
                elif isinstance(self._last_task, HeightRecoveryTask):
                    self._state = RobotStates.RECOVERY_FAILED
                elif isinstance(self._last_task, BlockRoombaTask):
                    self._state = RobotStates.WAITING_ON_RECOVERY
                # Hit handles its own abort due to the tricky situation
                elif isinstance(self._last_task, HitRoombaTask):
                    self._state = RobotStates.NORMAL

            # The task returned something that can't be interpreted
            else:
                rospy.logerr('Invalid ending task state provided in StateMonitor')
                self._state = RobotStates.FATAL

            rospy.loginfo('RobotState: ' + str(self._state))

            if self._state == RobotStates.FATAL:
                raise IARCFatalSafetyException('StateMonitor determined robot is in a fatal state')

    # fills out the rest of Intermediary State for the task
    def fill_out_transition(self, state):
        with self._lock:
            state.drone_odometry = self._drone_odometry
            state.roombas = self._roombas
            state.obstacles = self._obstacles
            state.arm_status = self._arm_status
            return state

    # Handles no task running timeouts
    def get_timeout_twist(self):
        twist = TwistStamped()
        twist.header.stamp = rospy.Time.now()
        return twist


    def signal_safety_active(self):
        with self._lock:
            self._state = RobotStates.SAFETY_ACTIVE

    def wait_until_ready(self, startup_timeout):
        while (rospy.Time.now() == rospy.Time(0) and not rospy.is_shutdown()):
            # Wait for time to be initialized
            rospy.sleep(0.005)
        if rospy.is_shutdown():
            raise rospy.ROSInterruptException()

        start_time = rospy.Time.now()

        while ((self._drone_odometry is None
                or self._roombas is None
                or self._obstacles is None
                or self._arm_status is None)
               and rospy.Time.now() < start_time + startup_timeout
               and not rospy.is_shutdown()):
            rospy.sleep(0.005)
        if rospy.Time.now() >= start_time + startup_timeout:
            rospy.logerr('Drone odometry: %s', self._drone_odometry is not None)
            rospy.logerr('Roombas: %s', self._roombas is not None)
            rospy.logerr('Obstacles: %s', self._obstacles is not None)
            rospy.logerr('Arm status: %s', self._arm_status is not None)
            raise IARCFatalSafetyException('SafetyMonitor timed out on startup')
        if rospy.is_shutdown():
            raise rospy.ROSInterruptException()

    """
    Callbacks for publishers

    Types:
        drone odometry: odometry (position, velocity, etc.) of drone
        roomba status: odometry (position, velocity, etc.) of
            all roombas in sight of drone
    """
    def _receive_drone_odometry(self, data):
        with self._lock:
            self._drone_odometry = data
            self._BELOW_MIN_MAN_HEIGHT = (data.pose.pose.position.z
                            < self._MIN_MANEUVER_HEIGHT)

    def _receive_roomba_status(self, data):
        with self._lock:
            self._roombas = data

    def _receive_obstacle_status(self, data):
        with self._lock:
            self._obstacles = data

    def _receive_arm_status(self, data):
        with self._lock:
            self._arm_status = data

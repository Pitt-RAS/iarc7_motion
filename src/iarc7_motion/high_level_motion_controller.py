#!/usr/bin/env python
import actionlib
import rospy
import sys
import threading
import traceback

from actionlib_msgs.msg import GoalStatus
from std_srvs.srv import SetBool

from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from iarc7_msgs.msg import TwistStampedArray, OdometryArray
from iarc7_msgs.msg import OrientationThrottleStamped, FlightControllerStatus

from iarc7_safety.SafetyClient import SafetyClient

from iarc7_motion.msg import QuadMoveGoal, QuadMoveAction
from iarc7_motion.msg import GroundInteractionGoal, GroundInteractionAction

from task_command_handler import TaskCommandHandler
from intermediary_state import IntermediaryState
from iarc_task_action_server import IarcTaskActionServer

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

import iarc_tasks.task_states as task_states
import iarc_tasks.task_commands as task_commands


class HighLevelMotionController:

    def __init__(self, action_server):
        # action server for getting requests from AI
        self._action_server = action_server

        # state data
        self._roombas = None
        self._drone_odometry = None
        self._arm_status = None
        
        # current state of HLM
        self._task = None
        self._initialized = False
        self._waiting_on_takeoff = True
        self._waiting_on_recovery = False

        # used for timeouts & extrapolating 
        self._timer = None
        self._last_twist = None
        self._timeout_vel_sent = False 

        # to keep things thread safe
        self._lock = threading.RLock()

        # handles communicating between tasks and LLM
        self._task_command_handler = TaskCommandHandler()

        # safety 
        self._safety_client = SafetyClient('high_level_motion')
        self._safety_land_complete = False
        self._safety_land_requested = False

        # Create action client to request a safety landing
        self._action_client = actionlib.SimpleActionClient(
                                        "motion_planner_server",
                                        QuadMoveAction)
        
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
            # update rate for HLM
            self._update_rate = rospy.get_param('~update_rate')
            # task timeout values
            self._task_timeout = rospy.Duration(rospy.get_param('~task_timeout'))
            self._task_timeout_deceleration_time = rospy.Duration(
                rospy.get_param('~task_timeout_deceleration_time'))
            self._MIN_MANEUVER_HEIGHT = rospy.get_param('~min_maneuver_height')

        except KeyError as e:
            rospy.logerr('Could not lookup a parameter for High Level Motion Controller')
            raise

    def run(self):
        # rate limiting of updates of HLM
        rate = rospy.Rate(self._update_rate)

        rospy.logwarn('trying to form bond')

        # forming bond with safety client
        if not self._safety_client.form_bond():
            rospy.logerr('High Level Motion could not form bond with safety client')
            return

        rospy.logwarn('done forming bond')

        self._action_client.wait_for_server()

        while not rospy.is_shutdown():
            # Exit immediately if fatal
            if self._safety_client.is_fatal_active() or self._safety_land_complete:
                return

            # Land if put into safety mode
            if self._safety_client.is_safety_active() and not self._safety_land_requested:
                # Request landing
                goal = QuadMoveGoal(movement_type="land", preempt=True)
                self._action_client.send_goal(goal,
                        done_cb=self._safety_task_complete_callback)
                rospy.logwarn('motion planner attempting to execute safety land')
                self._safety_land_requested = True

            # if we have not seen a task yet
            # we wait until now to start the task timeout timer
            # because the action clients have not yet been initialized
            if not self._initialized:
                self._timer = rospy.Timer(self._task_timeout, self._recieve_task_timeout)
                self._initialized = True

            if self._task is None:
                if self._action_server.has_new_task():
                    new_task = self._action_server.get_new_task()

                    if self._check_transition(new_task):
                        self._task = new_task
                        self._shutdown_timer()
                        self._task_command_handler.new_task(new_task, self._get_transition())
                    else: 
                        rospy.logwarn('AI provided illegal task transition. Aborting requested task.')
                        self._action_server.set_aborted()
            else: 
                if self._action_server.is_canceled():
                    self._task_command_handler.cancel_task()

                self._task_command_handler.run()
                task_state = self._task_command_handler.get_state()

                # handles state of task and HLM controller
                if isinstance(task_state, task_states.TaskCanceled):
                    self._action_server.set_canceled()
                    rospy.logwarn('Task was canceled')
                    self._task = None
                elif isinstance(task_state, task_states.TaskAborted):
                    rospy.logwarn('Task aborted with: %s', task_state.msg)
                    self._action_server.set_aborted()
                    self._task = None
                elif isinstance(task_state, task_states.TaskFailed):
                    rospy.logwarn('Task failed with: %s', task_state.msg)
                    self._action_server.set_succeeded(False)
                    self._task = None
                elif isinstance(task_state, task_states.TaskDone):
                    self._action_server.set_succeeded(True)
                    self._task = None
                else:
                    assert isinstance(task_state, task_states.TaskRunning)

                self._last_twist = self._task_command_handler.get_last_twist()

                # as soon as we set a task to None, start timeout timer
                if self._task is None:
                    self._timer = rospy.Timer(self._task_timeout, self._recieve_task_timeout)
                    self._timeout_vel_sent = False

            rate.sleep()

    def _recieve_task_timeout(self, event):
        """
        Handles no task running timeouts

        Args: 
            event: rospy.TimerEvent 
                (see http://wiki.ros.org/rospy/Overview/Time)
        """
        with self._lock:
            if self._task is None: 
                if not self._timeout_vel_sent:
                    commands = TwistStampedArray()

                    if self._last_twist is None:
                        self._last_twist = TwistStamped()
                    self._last_twist.header.stamp = rospy.Time.now()

                    # send future twist with zero velocity
                    # so that LLM will interpolate between the 
                    # current velocity and 0m/s to avoid impulse in velocity
                    future_twist = TwistStamped()
                    future_twist.header.stamp = (rospy.Time.now() + 
                        self._task_timeout_deceleration_time)

                    commands.twists = [self._last_twist, future_twist]

                    # if timed-out, send twists to LLM
                    self._task_command_handler.send_timeout(commands)
                    self._timeout_vel_sent = True
                    self._last_twist = future_twist
                rospy.logwarn('Task running timeout. Setting zero velocity')
            else: 
                rospy.logerr('Timeout timer in HLM fired with task running')

    # checks task transitions before executing it
    def _check_transition(self, task):
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

    # fills out the Intermediary State for the task
    def _get_transition(self):
        while ((self._drone_odometry is None or self._roombas is None 
                or self._arm_status is None) and not rospy.is_shutdown()):
            pass

        with self._lock:
            state = IntermediaryState() 
            state.drone_odometry = self._drone_odometry
            state.roombas = self._roombas
            state.timeout_sent = self._timeout_vel_sent
            state.last_task_ending_state = self._task_command_handler.get_state()
            state.arm_status = self._arm_status
            state.last_twist = self._last_twist
            return state

    # shuts down the timer so the callback is not called when a task is running
    def _shutdown_timer(self):
        with self._lock:
            self._timer.shutdown()
            self._timer = None

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

    # callback for safety task completition
    def _safety_task_complete_callback(self, status, response):
        with self._lock: 
            if response.success:
                rospy.logwarn('High Level Motion supposedly safely landed the aircraft')
            else:
                rospy.logerr('High Level Motion did not safely land aircraft')
            self._safety_land_complete = True

if __name__ == '__main__':
    rospy.init_node('high_level_motion')

    # action server for getting requests from AI
    action_server = IarcTaskActionServer()
    high_level_motion_controller = HighLevelMotionController(action_server)
    
    try:
        high_level_motion_controller.run()
    except Exception, e:
        rospy.logfatal("Error in High Level Motion while running.")
        rospy.logfatal(str(e))
        rospy.logfatal(traceback.format_exc())
        raise
    finally:
        rospy.signal_shutdown("High Level Motion shutdown")
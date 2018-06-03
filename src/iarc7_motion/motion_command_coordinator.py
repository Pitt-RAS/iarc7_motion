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
from iarc7_safety.iarc_safety_exception import (IARCSafetyException,
                                                IARCFatalSafetyException)

from iarc7_motion.msg import QuadMoveGoal, QuadMoveAction
from iarc7_motion.msg import GroundInteractionGoal, GroundInteractionAction

from task_command_handler import TaskCommandHandler
from state_monitor import StateMonitor
from transition_data import TransitionData
from iarc_task_action_server import IarcTaskActionServer

import iarc_tasks.task_states as task_states
import iarc_tasks.task_commands as task_commands


class MotionCommandCoordinator:

    def __init__(self, action_server):
        # action server for getting requests from AI
        self._action_server = action_server
        
        # current state of motion coordinator
        self._task = None
        self._first_task_seen = False

        # used for timeouts & extrapolating 
        self._time_of_last_task = None
        self._timeout_vel_sent = False 

        # to keep things thread safe
        self._lock = threading.RLock()

        # handles monitoring of state of drone
        self._state_monitor = StateMonitor()

        # handles communicating between tasks and LLM
        self._task_command_handler = TaskCommandHandler()

        # safety 
        self._safety_client = SafetyClient('motion_command_coordinator')
        self._safety_land_complete = False
        self._safety_land_requested = False

        # Create action client to request a safety landing
        self._action_client = actionlib.SimpleActionClient(
                                        "motion_planner_server",
                                        QuadMoveAction)
        try:
            # update rate for motion coordinator
            self._update_rate = rospy.get_param('~update_rate')
            # task timeout values
            self._task_timeout = rospy.Duration(rospy.get_param('~task_timeout'))
            # startup timeout
            self._startup_timeout = rospy.Duration(rospy.get_param('~startup_timeout'))
        except KeyError as e:
            rospy.logerr('Could not lookup a parameter for motion coordinator')
            raise

    def run(self):
        # rate limiting of updates of motion coordinator
        rate = rospy.Rate(self._update_rate)

        # waiting for dependencies to be ready
        while rospy.Time.now() == rospy.Time(0) and not rospy.is_shutdown():
            rospy.sleep(0.005)
        if rospy.is_shutdown():
            raise rospy.ROSInterruptException()
        start_time = rospy.Time.now()
        if not self._action_client.wait_for_server(self._startup_timeout):
            raise IARCFatalSafetyException(
                    'Motion Coordinator could not initialize action client')
        self._state_monitor.wait_until_ready(self._startup_timeout
                                           - (rospy.Time.now() - start_time))

        # forming bond with safety client
        if not self._safety_client.form_bond():
            raise IARCFatalSafetyException('Motion Coordinator could not form bond with safety client')

        while not rospy.is_shutdown():
            with self._lock:
                # Exit immediately if fatal
                if self._safety_client.is_fatal_active():
                    raise IARCFatalSafetyException('Safety Client is fatal active') 
                elif self._safety_land_complete:
                    return

                # Land if put into safety mode
                if self._safety_client.is_safety_active() and not self._safety_land_requested:
                    # Request landing
                    goal = QuadMoveGoal(movement_type="land", preempt=True)
                    self._action_client.send_goal(goal,
                            done_cb=self._safety_task_complete_callback)
                    rospy.logwarn('motion coordinator attempting to execute safety land')
                    self._safety_land_requested = True
                    self._state_monitor.signal_safety_active()

                # set the time of last task to now if we have not seen a task yet
                if not self._first_task_seen:
                    self._time_of_last_task = rospy.Time.now()
                    self._first_task_seen = True

                if self._task is None and self._action_server.has_new_task():
                    new_task = self._action_server.get_new_task()

                    if self._state_monitor.check_transition(new_task):
                        self._time_of_last_task = None
                        self._task = new_task
                        self._task_command_handler.new_task(new_task, self._get_current_transition())
                    else: 
                        rospy.logwarn('Illegal task transition request requested in motion coordinator. Aborting requested task.')
                        self._action_server.set_aborted()
                
                if self._task is not None: 
                    no_error_canceling = True
                    if self._action_server.is_canceled():
                        no_error_canceling = self._task_command_handler.cancel_task()

                    # if canceling task did not result in an error
                    if no_error_canceling:
                        self._task_command_handler.run()

                    task_state = self._task_command_handler.get_state()

                    # handles state of task, motion coordinator, and action server
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
                    elif not isinstance(task_state, task_states.TaskRunning):
                        rospy.logerr("Invalid task state returned, aborting task")
                        self._action_server.set_aborted()
                        self._task = None
                        task_state = task_states.TaskAborted(msg='Invalid task state returned')

                    # as soon as we set a task to None, start time
                    # and send ending state to State Monitor
                    if self._task is None:
                        self._time_of_last_task = rospy.Time.now()
                        self._timeout_vel_sent = False
                        self._state_monitor.set_last_task_end_state(task_state)
                else: 
                    if (rospy.Time.now() - self._time_of_last_task) > self._task_timeout:
                        self._receive_task_timeout()

                rate.sleep()

    # fills out the Intermediary State for the task
    def _get_current_transition(self):
        state = TransitionData() 
        state.last_twist = self._task_command_handler.get_last_twist()
        state.last_task_ending_state = self._task_command_handler.get_state()
        state.timeout_sent = self._timeout_vel_sent
        return self._state_monitor.fill_out_transition(state)

    # callback for safety task completition
    def _safety_task_complete_callback(self, status, response):
        with self._lock: 
            if response.success:
                rospy.logwarn('Motion Coordinator supposedly safely landed the aircraft')
            else:
                rospy.logerr('Motion Coordinator did not safely land aircraft')
            self._safety_land_complete = True

    def _receive_task_timeout(self):
        """
        Handles no task running timeouts
        """
        with self._lock:
            # task should be None when this callback is called
            if self._task is None: 
                # if we have not sent a timeout velocity yet
                if not self._timeout_vel_sent:
                    # calls public method in Task Command Handler to 
                    # publish twist stamped array, which came from 
                    # the State Monitor
                    self._task_command_handler.send_timeout(
                        self._state_monitor.get_timeout_twist())

                    self._timeout_vel_sent = True

                rospy.logwarn_throttle(1.0, 'Task running timeout. Setting zero velocity')
            else: 
                raise IARCFatalSafetyException('Timeout called in motion coordinator with task running')

if __name__ == '__main__':
    rospy.init_node('motion_command_coordinator')

    # action server for getting action/motion requests
    action_server = IarcTaskActionServer()
    motion_command_coordinator = MotionCommandCoordinator(action_server)
    
    try:
        motion_command_coordinator.run()
    except Exception, e:
        rospy.logfatal("Error in Motion Command Coordinator while running.")
        rospy.logfatal(str(e))
        rospy.logfatal(traceback.format_exc())
        raise
    finally:
        rospy.signal_shutdown("Motion Coordinator shutdown")

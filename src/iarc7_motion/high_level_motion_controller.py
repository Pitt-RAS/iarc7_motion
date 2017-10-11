#!/usr/bin/env python

import math
import rospy
import tf2_ros
import tf2_geometry_msgs
import threading

from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Twist

from iarc7_motion.msg import QuadMoveGoal, QuadMoveAction
from iarc7_msgs.msg import TwistStampedArray, OrientationThrottleStamped

import iarc_tasks.task_states as task_states
import iarc_tasks.task_commands as task_commands

from task_command_handler import TaskCommandHandler
from intermediary_state import IntermediaryState

class HighLevelMotionController:

    def __init__(self, action_server):
        # action server for getting requests from AI
        self._action_server = action_server

        # state data
        self._roombas = None
        self._drone_odometry = None
        
        # current state of HLM
        self._task = None
        self._initialized = False

        # used for timeouts & extrapolating 
        self._timer = None
        self._last_twist = None
        self._timeout_vel_sent = False 

        # to keep things thread safe
        self._lock = threading.RLock()

        # handles communicating between tasks and LLM
        self._task_command_handler = TaskCommandHandler()

        # safety 
        self._safety_client = SafetyClient('motion_planner')
        self._safety_land_complete = False
        self._safety_land_requested = False

        # Create action client to request a safety landing
        self._action_client = actionlib.SimpleActionClient(
                                        "motion_planner_server",
                                        QuadMoveAction)
        
        # needed to do sanity checking and state monitoring
        self._roomba_status_sub = rospy.Subscriber(
            'roombas', OdometryArray, 
            self._receive_roomba_status)

        self._current_velocity_sub = rospy.Subscriber(
            '/odometry/filtered', Odometry,
            self._recieve_drone_odometry)

        try:
            # update rate for HLM
            self._update_rate = rospy.get_param('~update_rate')
            # task timeout values
            self._task_timeout = rospy.Duration(rospy.get_param('~task_timeout'))
            self._task_timeout_deceleration_time = rospy.Duration(rospy.get_param('~task_timeout_deceleration_time'))

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
                rospy.logwarn(
                        'motion planner attempting to execute safety land')
                self._safety_land_requested = True

            # if we have not seen a task yet
            # we wait until now to start the task timeout timer
            # because the action clients have not yet been initialized
            if not self._initialized:
                self._timer = rospy.Timer(self._task_timeout, self._recieve_task_timeout)
                self._initialized = True

            if (self._task is None):
                if self._action_server.has_new_task():
                    new_task = self._action_server.get_new_task()

                    if self._check_transition(new_task):
                        self._task = new_task
                        self._shutdown_timer()
                        self._task_command_handler.new_task(new_task, self._get_transition())
                    else: 
                        rospy.logwarn('AI provided illegal task transition. Aborting requested task.')
                        self._action_server.set_aborted()
                        self._task = None
            else: 
                task_state = self._task_command_handler.get_state()

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

                if self._task is None:
                    self._timer = rospy.Timer(self._task_timeout, self._recieve_task_timeout)

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
                    self._task_command_handler._publish_twist(commands)

                    self._timeout_vel_sent = True
                rospy.logwarn('Task running timeout. Setting zero velocity')
            else: 
                rospy.logerr('Timeout timer in HLM fired with task running')

    def _check_transition(self):
        return True

    def _get_transition(self):
        return IntermediaryState()

    def _shutdown_timer(self):
        with self._lock:
            self._timer.shutdown()
            self._timer = None

    def _recieve_drone_odometry(self, data):
        with self._lock:
            self._drone_odometry = data

    def _receive_roomba_status(self, data):
        with self._lock:
            self._roombas = data

    def _safety_task_complete_callback(self, status, response):
        with self._lock: 
            if response.success:
                rospy.logwarn('High Level Motion supposedly safely landed the aircraft')
            else:
                rospy.logerr('High Level Motion did not safely land aircraft')
            self._safety_land_complete = True

if __name__ == '__main__':
    rospy.init_node('high_level_motion_controller')

    # action server for getting requests from AI
    action_server = IarcTaskActionServer()
    high_level_motion_controller = HighLevelMotionController(action_server)
    
    try:
        high_level_motion_controller.run()
    except Exception, e:
        import traceback
        rospy.logfatal("Error in High Level Motion while running.")
        rospy.logfatal(str(e))
        rospy.logfatal(traceback.format_exc())
        raise
    finally:
        rospy.signal_shutdown("High Level Motion shutdown")
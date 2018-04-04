#!/usr/bin/env python

import sys
import traceback
import actionlib
import rospy

import iarc_tasks.task_states as task_states
import iarc_tasks.task_commands as task_commands

from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped

from nav_msgs.msg import Path

from iarc7_msgs.msg import MotionPointStamped, MotionPointStampedArray

from iarc7_safety.iarc_safety_exception import IARCFatalSafetyException

from iarc7_msgs.msg import TwistStampedArray, OrientationThrottleStamped
from iarc7_motion.msg import GroundInteractionGoal, GroundInteractionAction

from linear_motion_profile_generator import LinearMotionProfileGenerator

class TaskCommandHandler:

    def __init__(self):
        # class state
        self._task = None
        self._last_task_commands = None
        self._in_passthrough = False
        self._task_state = None
        self._transition = None
        self._last_twist = None

        self._ground_interaction_task_callback = None

        # Creates the SimpleActionClient for requesting ground interaction
        self._ground_interaction_client = actionlib.SimpleActionClient(
                                          'ground_interaction_action',
                                          GroundInteractionAction)

        # used to send velocity commands to LLM
        self._motion_point_pub = rospy.Publisher('motion_point_targets',
                                                 MotionPointStampedArray,
                                                 queue_size=0)

        # Used to send Path messages for visualizatio in Rviz
        self._local_plan_pub = rospy.Publisher('local_plan',
                                               Path,
                                               queue_size=0)

        # used to send passthrough commands to LLM
        self._passthrough_pub = rospy.Publisher('passthrough_command',
                                                OrientationThrottleStamped,
                                                queue_size=10)

        # task commands that are implemented and available to tasks
        self._command_implementations = {
            task_commands.VelocityCommand: self._handle_velocity_command,
            task_commands.NopCommand: self._handle_nop_command,
            task_commands.GroundInteractionCommand: self._handle_ground_interaction_command,
            task_commands.ConfigurePassthroughMode: self._handle_passthrough_mode_command,
            task_commands.AngleThrottleCommand: self._handle_passthrough_command
            }

        self._motion_profile_generator = LinearMotionProfileGenerator.get_linear_motion_profile_generator()

    # takes in new task from HLM Controller
    # transition is of type TransitionData 
    def new_task(self, task, transition):
        self._task = task
        self._transition = transition
        self._task_state = task_states.TaskRunning()
        self._task.set_incoming_transition(self._transition)

    # cancels task
    def cancel_task(self):
        if self._task is None:
            raise IARCFatalSafetyException('No task running to cancel')
        try:
            ready = self._task.cancel()
            if ready: 
                self._task = None
                self._task_state = task_states.TaskCanceled()
            else: 
                rospy.logwarn('Task has not completed')
            return True
        except Exception as e:
            rospy.logerr('Exception canceling task')
            rospy.logerr(str(e))
            rospy.logerr(traceback.format_exc())
            rospy.logerr('Task Command Handler aborted task')
            self._task_state = task_states.TaskAborted(msg='Error canceling task')
            self._task = None
            return False

    # returns last command (ground interaction, velocity, etc.)
    # that task returned
    def get_last_command(self):
        return self._last_task_commands

    # returns last twist sent to LLM
    def get_last_twist(self):
        return self._last_twist
    
    # public method wrapper for getting task state
    def get_state(self):
        return self._task_state

    # main function
    def run(self):
        task_commands = self._get_task_command()
        for task_command in task_commands:
            try:
                self._command_implementations[type(task_command)](task_command)
            except (KeyError, TypeError) as e:
                rospy.logerr("Task requested unimplemented command, noping: %s", type(task_command))
                self._handle_nop_command(None)

        self._last_task_commands = task_commands

    # gets desired command from running task
    def _get_task_command(self):
        if self._task is not None:
            try:
                task_request = self._task.get_desired_command()
            except Exception as e:
                rospy.logerr('Exception getting task command')
                rospy.logerr(str(e))
                rospy.logerr(traceback.format_exc())
                rospy.logerr('Task Command Handler aborted task')
                self._task = None
                self._task_state = task_states.TaskAborted(msg='Exception getting task command')
                return (task_commands.NopCommand(),)

            try: 
                _task_state = task_request[0]
            except (TypeError, IndexError) as e: 
                rospy.logerr('Exception getting task state')
                rospy.logerr(str(e))
                rospy.logerr(traceback.format_exc())
                rospy.logerr('Task Command Handler aborted task')
                self._task = None
                self._task_state = task_states.TaskAborted(msg='Exception getting task state')
                return (task_commands.NopCommand(),)
            
            if issubclass(type(_task_state), task_states.TaskState):
                self._task_state = _task_state
            else: 
                rospy.logerr('Task provided unknown state')
                rospy.logerr('Task Command Handler aborted task')
                self._task = None
                self._task_state = task_states.TaskAborted(msg='Error getting task state')
                return (task_commands.NopCommand(),)
            
            try: 
                if isinstance(self._task_state, task_states.TaskDone):
                    self._task = None
                    return task_request[1:]
                elif isinstance(self._task_state, task_states.TaskRunning):
                    return task_request[1:]
                else: 
                    self._task = None
            except (TypeError, IndexError) as e: 
                rospy.logerr('Exception getting task request')
                rospy.logerr(str(e))
                rospy.logerr(traceback.format_exc())
                rospy.logerr('Task Command Handler aborted task')
                self._task = None
                self._task_state = task_states.TaskAborted(msg='Exception getting task request')
                return (task_commands.NopCommand(),)
        elif isinstance(self._task_state, task_states.TaskCanceled):
            pass
        else: 
            raise IARCFatalSafetyException('TaskCommandHandler ran with no task running.')

        # no action to take, return a Nop
        return (task_commands.NopCommand(),)

    """
    Command Handlers

    Types: 
        ground interaction: these include takeoff
        nop command: do nothing
        velocity command: requests a velocity (x, y, z, and angular)
        passthrough command: passthrough to throttle, pitch, and roll commands

    """
    def _handle_nop_command(self, nop_command):
        pass

    def _handle_velocity_command(self, velocity_command):
        plan, pose_only_plan = self._motion_profile_generator.get_velocity_plan(velocity_command)
        self._publish_motion_profile(plan, pose_only_plan)

    def _handle_passthrough_mode_command(self, passthrough_mode_command):
        if passthrough_mode_command.enable:
            if self._ground_interaction_task_callback is not None:
                rospy.logerr('Task requested a passthrough action before the last was completed')
                rospy.logerr('Task Command Handler aborted task')
                self._task = None
                self._task_state = task_states.TaskAborted(
                        msg='Task requested a passthrough action before the last was completed')
                self._ground_interaction_task_callback = None
                self._ground_interaction_client.cancel_goal()
                self._ground_interaction_client.stop_tracking_goal()
                return

            self._ground_interaction_task_callback = passthrough_mode_command.completion_callback
            goal = GroundInteractionGoal(interaction_type='passthrough')
            self._ground_interaction_client.send_goal(goal, done_cb=self.handle_ground_interaction_done)
            self._in_passthrough = True
        else:
            self._ground_interaction_client.cancel_goal()
            self._in_passthrough = False

    def _handle_passthrough_command(self, passthrough_command):
        if self._in_passthrough:
            msg = OrientationThrottleStamped()
            msg.header.stamp = rospy.Time.now()
            msg.data.pitch = passthrough_command.pitch
            msg.data.roll = passthrough_command.roll
            msg.data.yaw = passthrough_command.vyaw
            msg.throttle = passthrough_command.vz
            self._passthrough_pub.publish(msg)
        else:
            rospy.logerr('Task requested a passthrough command when not in passthrough mode')

    def _handle_ground_interaction_command(self, ground_interaction_command):
        if self._ground_interaction_task_callback is not None:
            rospy.logerr('Task requested another ground interaction action before the last completed')
            rospy.logerr('Task Command Handler aborted task')
            self._task = None
            self._task_state = task_states.TaskAborted(
                        msg='Task requested another ground interaction action before the last completed')
            self._ground_interaction_task_callback = None
            self._ground_interaction_client.cancel_goal()
            self._ground_interaction_client.stop_tracking_goal()
            return

        self._ground_interaction_task_callback = ground_interaction_command.completion_callback
        # Request ground interaction of llm
        goal = GroundInteractionGoal(interaction_type=ground_interaction_command.interaction_type)
        # Sends the goal to the action server.
        self._ground_interaction_client.send_goal(goal, done_cb=self.handle_ground_interaction_done)
    
    # callback for status on ground interaction commands
    def handle_ground_interaction_done(self, status, result):
        if self._ground_interaction_task_callback is not None:
            try:
                self._ground_interaction_task_callback(status, result)
            except Exception as e:
                rospy.logerr('Exception sending result using ground interaction done cb')
                rospy.logerr('Task Command Handler aborted task')
                rospy.logerr(str(e))
                rospy.logerr(traceback.format_exc())
                self._task = None
                self._task_state = task_states.TaskAborted(
                        msg='Exception sending result using ground interaction done cb')
            self._ground_interaction_task_callback = None
        else:
            rospy.logerr('Ground interaction done callback received with no task callback available')

    """
    Sends motion point stamped array to LLM

    Args:
        motion_point_stamped_array: MotionPointStampedArray
    """
    def _publish_motion_profile(self, motion_point_stamped_array, path):
        self._last_twist = motion_point_stamped_array.motion_points[-1].motion_point.twist
        self._local_plan_pub.publish(path)
        self._motion_point_pub.publish(motion_point_stamped_array)

    # public wrapper for HLM Controller to send timeouts
    def send_timeout(self, twist):
        self._handle_velocity_command(task_commands.VelocityCommand(twist))

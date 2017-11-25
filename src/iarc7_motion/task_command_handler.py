#!/usr/bin/env python

import sys
import traceback
import actionlib
import rospy

import iarc_tasks.task_states as task_states
import iarc_tasks.task_commands as task_commands

from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped

from iarc7_msgs.msg import TwistStampedArray, OrientationThrottleStamped
from iarc7_motion.msg import GroundInteractionGoal, GroundInteractionAction

class TaskCommandHandler:

    def __init__(self):
        # class state
        self._task = None
        self._last_task_commands = None
        self._in_passthrough = False
        self._task_state = None
        self._transition = None
        self._task_canceled = False
        self._last_twist = None

        self._ground_interaction_task_callback = None

        # Creates the SimpleActionClient for requesting ground interaction
        self._ground_interaction_client = actionlib.SimpleActionClient(
                                          'ground_interaction_action',
                                          GroundInteractionAction)

        # used to send velocity commands to LLM
        self._velocity_pub = rospy.Publisher('movement_velocity_targets',
                                             TwistStampedArray,
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

    # takes in new task from HLM Controller
    # transition is of type TransitionData 
    def new_task(self, task, transition):
        self._task = task
        self._transition = transition
        self._task_canceled = False
        self._task_state = task_states.TaskRunning()
        self._task.send_transition(self._transition)

    # cancels task
    def cancel_task(self):
        if self._task is None:
            rospy.logerr('No task running to cancel.')
        try:
            self._task.cancel()
            self._task_canceled = True
            self._task = None
        except Exception as e:
            rospy.logerr('Exception canceling task')
            rospy.logerr(str(e))
            rospy.logerr(traceback.format_exc())
            rospy.logerr('Task Command Handler aborted task')
            self._task_state = task_states.TaskAborted()
            self._task = None

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
                rospy.logerr('Exception getting tasks preferred velocity')
                rospy.logerr(str(e))
                rospy.logerr(traceback.format_exc())
                rospy.logerr('Task Command Handler aborted task')
                self._task = None
                self._task_state = task_states.TaskAborted()
                return (task_commands.NopCommand(),)

            self._task_state = task_request[0]

            if isinstance(self._task_state, task_states.TaskDone):
                self._task = None
                return task_request[1:]
            elif isinstance(self._task_state, task_states.TaskRunning):
                return task_request[1:]
            else: 
                self._task = None
        elif self._task_canceled:
            self._task_state = task_states.TaskCanceled()
        else: 
            self._task_state = task_states.TaskAborted()
            rospy.logerr('TaskCommandHandler ran with no task running. Setting task state to aborted.')

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
        self._publish_twist(velocity_command.target_twist)

    def _handle_passthrough_mode_command(self, passthrough_mode_command):
        if passthrough_mode_command.enable:
            if self._ground_interaction_task_callback is not None:
                rospy.logerr('Task requested a passthrough action before the last was completed')
                rospy.logerr('Task Command Handler aborted task')
                self._task = None
                self._task_state = task_states.TaskAborted()
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
            self._task_state = task_states.TaskAborted()
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
                self._task_state = task_states.TaskAborted()
            self._ground_interaction_task_callback = None
        else:
            rospy.logerr('Ground interaction done callback received with no task callback available')

    """
    Sends twist (x, y, z, and angular velocities) to LLM

    Args:
        twist: TwistStampedArray (array of twists) or just a single twist stamped.
    """
    def _publish_twist(self, twist):
        if type(twist) is TwistStampedArray:
            self._last_twist = twist.twists[-1]
            self._velocity_pub.publish(twist)
        elif type(twist) is TwistStamped:
            self._last_twist = twist
            velocity_msg = TwistStampedArray()
            velocity_msg.twists = [twist]
            self._velocity_pub.publish(velocity_msg)
        else:
            raise TypeError('Illegal type provided in Task Command Handler')

    # public wrapper for HLM Controller to send timeouts
    def send_timeout(self, twist):
        self._publish_twist(twist)

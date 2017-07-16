#!/usr/bin/env python

import sys
import traceback
import actionlib
import rospy

from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import TwistStamped

from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool

from iarc7_motion.msg import GroundInteractionGoal, GroundInteractionAction
from iarc7_motion.msg import QuadMoveGoal, QuadMoveAction
from iarc7_msgs.msg import TwistStampedArray, OrientationThrottleStamped
from iarc7_msgs.srv import Arm
from iarc7_safety.SafetyClient import SafetyClient

from iarc_task_action_server import IarcTaskActionServer

import iarc_tasks.task_states as task_states
import iarc_tasks.task_commands as task_commands

class MotionPlanner:

    def __init__(self, _action_server, update_rate):
        self._action_server = _action_server
        self._update_rate = update_rate
        self._task = None
        self._velocity_pub = rospy.Publisher('movement_velocity_targets',
                                             TwistStampedArray,
                                             queue_size=0)

        self._in_passthrough = False
        self._passthrough_pub = rospy.Publisher('passthrough_command',
                                                OrientationThrottleStamped,
                                                queue_size=10)

        self._arm_service = rospy.ServiceProxy('uav_arm', Arm)
        self._safety_client = SafetyClient('motion_planner')
        self._safety_land_complete = False
        self._safety_land_requested = False
        # Create action client to request a landing
        self._action_client = actionlib.SimpleActionClient(
                                        "motion_planner_server",
                                        QuadMoveAction)

        # Creates the SimpleActionClient for requesting ground interaction
        self._ground_interaction_client = actionlib.SimpleActionClient(
                                          'ground_interaction_action',
                                          GroundInteractionAction)

        self._ground_interaction_task_callback = None

        self._command_implementations = {
            task_commands.VelocityCommand: self._handle_velocity_command,
            task_commands.ArmCommand: self._handle_arm_command,
            task_commands.NopCommand: self._handle_nop_command,
            task_commands.GroundInteractionCommand: self._handle_ground_interaction_command,
            task_commands.ConfigurePassthroughMode: self._handle_passthrough_mode_command,
            task_commands.AngleThrottleCommand: self._handle_passthrough_command
            }

        self._time_of_last_task = None
        self._last_twist = None
        try:
            self._task_timeout = rospy.Duration(rospy.get_param('~task_timeout'))
        except KeyError as e:
            rospy.logerr('Could not lookup a parameter for motion_planner')
            raise

    def run(self):
        rate = rospy.Rate(self._update_rate)

        rospy.logwarn('trying to form bond')
        if not self._safety_client.form_bond():
            rospy.logerr('Motion planner could not form bond with safety client')
            return
        rospy.logwarn('done forming bond')

        rospy.wait_for_service('uav_arm')
        self._action_client.wait_for_server()
        self._ground_interaction_client.wait_for_server()

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

            task_commands = self._get_task_command()
            for task_command in task_commands:
                try:
                    self._command_implementations[type(task_command)](task_command)
                except (KeyError, TypeError) as e:
                    rospy.logerr("Task requested unimplemented command, noping: %s", type(task_command))
                    self._handle_nop_command(None)

            rate.sleep()

    def handle_ground_interaction_done(self, status, result):
        if self._ground_interaction_task_callback is not None:
            try:
                self._ground_interaction_task_callback(status, result)
            except Exception as e:
                rospy.logerr('Exception sending result using ground interaction done cb')
                rospy.logerr(str(e))
                rospy.logerr(traceback.format_exc())
                rospy.logwarn('Motion planner aborted task')
                self._action_server.set_aborted()
                self._task = None
            self._ground_interaction_task_callback = None
        else:
            rospy.logerr('Ground interaction done callback received with no task callback available')

    def _handle_ground_interaction_command(self, ground_interaction_command):
        if self._ground_interaction_task_callback is not None:
            rospy.logerr('Task requested another ground interaction action before the last completed')
            rospy.logwarn('Motion planner aborted task')
            self._action_server.set_aborted()
            self._task = None
            self._ground_interaction_task_callback = None
            self._ground_interaction_client.cancel_goal()
            self._ground_interaction_client.stop_tracking_goal()
            return

        self._ground_interaction_task_callback = ground_interaction_command.completion_callback
        # Request ground interaction of llm
        goal = GroundInteractionGoal(interaction_type=ground_interaction_command.interaction_type)
        # Sends the goal to the action server.
        self._ground_interaction_client.send_goal(goal, done_cb=self.handle_ground_interaction_done)

    def _handle_nop_command(self, nop_command):
        pass

    def _handle_velocity_command(self, velocity_command):
        self._publish_twist(velocity_command.target_twist)

    def _handle_arm_command(self, arm_command):
        arm_result = self._use_arm_service(arm_command.arm_state, arm_command.set_mode, arm_command.angle)
        self._call_tasks_arm_service_callback(arm_command.completion_callback, arm_result)

    def _call_tasks_arm_service_callback(self, callback, arm_result):
        try:
            callback(arm_result)
        except Exception as e:
            rospy.logerr('Exception setting result using an arm callback')
            rospy.logerr(str(e))
            rospy.logerr(traceback.format_exc())
            rospy.logwarn('Motion planner aborted task')
            self._action_server.set_aborted()
            self._task = None

    def _use_arm_service(self, arm, set_mode, angle):
        try:
            armed = self._arm_service(arm, set_mode, angle)
        except rospy.ServiceException as exc:
            rospy.logerr("Could not arm: " + str(exc))
            armed = False
        return armed

    def _handle_passthrough_mode_command(self, passthrough_mode_command):
        if passthrough_mode_command.enable:
            if self._ground_interaction_task_callback is not None:
                rospy.logerr('Task requested a passthrough action before the last was completed')
                rospy.logwarn('Motion planner aborted task')
                self._action_server.set_aborted()
                self._task = None
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

    def _publish_twist(self, twist):

        if type(twist) is TwistStampedArray:
            self._last_twist = twist.twists[-1]
            self._velocity_pub.publish(velocity_msg)
        elif type(twist) is TwistStamped:
            self._last_twist = twist
            velocity_msg = TwistStampedArray()
            velocity_msg.twists = [twist]
            self._velocity_pub.publish(velocity_msg)
        else:
            raise TypeError('Illegal type provided in Motion Planner')

    def _safety_task_complete_callback(self, status, response):
        if response.success:
            rospy.logwarn('Motion planner supposedly safely landed the aircraft')
        else:
            rospy.logerr('Motion planner did not safely land aircraft')
        self._safety_land_complete = True

    def _get_task_command(self):
        if (self._task is None) and self._action_server.has_new_task():
            self._task = self._action_server.get_new_task()

        if self._task:
            self._time_of_last_task = None
            if self._action_server.is_canceled():
                try:
                    self._task.cancel()
                except Exception as e:
                    rospy.logerr('Exception canceling task')
                    rospy.logerr(str(e))
                    rospy.logerr(traceback.format_exc())
                    rospy.logwarn('Motion planner aborted task')
                    self._action_server.set_aborted()
                    self._task = None
                    return (task_commands.NopCommand(),)

            try:
                task_request = self._task.get_desired_command()
            except Exception as e:
                rospy.logerr('Exception getting tasks preferred velocity')
                rospy.logerr(str(e))
                rospy.logerr(traceback.format_exc())
                rospy.logwarn('Motion planner aborted task')
                self._action_server.set_aborted()
                self._task = None
                return (task_commands.NopCommand(),)

            task_state = task_request[0]
            if isinstance(task_state, task_states.TaskCanceled):
                self._action_server.set_canceled()
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
                return task_request[1:]
            else:
                assert isinstance(task_state, task_states.TaskRunning)
                return task_request[1:]
        else:
            # There is no current task running
            if self._time_of_last_task is None:
                self._time_of_last_task = rospy.Time.now()

            if ((rospy.Time.now() - self._time_of_last_task)
                > self._task_timeout):
                # If past the timeout send a zero velocity command
                self._handle_velocity_command(task_commands.VelocityCommand())
                rospy.logwarn('Task running timeout Setting zero velocity')
                # reset time so that a new command will be sent
                self._time_of_last_task = None

        # No action to take return a nop
        return (task_commands.NopCommand(),)

if __name__ == '__main__':
    rospy.init_node('motion_planner')
    action_server = IarcTaskActionServer()

    update_rate = rospy.get_param('~update_rate', False)

    motion_planner = MotionPlanner(action_server, update_rate)
    try:
        motion_planner.run()
    except Exception, e:
        import traceback
        rospy.logfatal("Error in motion planner while running.")
        rospy.logfatal(str(e))
        rospy.logfatal(traceback.format_exc())
        raise
    finally:
        rospy.signal_shutdown("Motion Planner shutdown")

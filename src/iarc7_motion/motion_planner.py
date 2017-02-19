#!/usr/bin/env python

import actionlib
import rospy
import sys
import traceback

from geometry_msgs.msg import TwistStamped
from std_srvs.srv import SetBool

from iarc7_motion.msg import QuadMoveGoal, QuadMoveAction
from iarc7_msgs.msg import TwistStampedArrayStamped
from iarc7_safety.SafetyClient import SafetyClient

from iarc_task_action_server import IarcTaskActionServer
from iarc_tasks.task_state import TaskState

class MotionPlanner:

    def __init__(self, _action_server):
        self._action_server = _action_server
        self._task = None
        self._velocity_pub = rospy.Publisher('movement_velocity_targets', TwistStampedArrayStamped, queue_size=0)
        self._arm_service = rospy.ServiceProxy('uav_arm', SetBool)
        self._safety_client = SafetyClient('motion_planner')
        self._safety_land_complete = False
        self._safety_land_requested = False
        # Create action client to request a landing
        self._action_client = actionlib.SimpleActionClient("motion_planner_server", QuadMoveAction)

    def run(self):
        rate = rospy.Rate(10)

        rospy.logwarn('trying to form bond')
        if not self._safety_client.form_bond():
            rospy.logerr('Motion planner could not form bond with safety client')
            return
        rospy.wait_for_service('uav_arm')
        rospy.logwarn('done forming bond')

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

            # Tuples could have different lengths so just get the whole tuple
            task_command = self._get_task_command()
            if(task_command[0] == 'velocity'):
                self._publish_twist(task_command[1])
            elif(task_command[0] == 'arm'):
                arm_result = self._use_arm_service(True)
                self._call_tasks_arm_service_callback(task_command[1], arm_result)
            elif(task_command[0] == 'disarm'):
                arm_result = self._use_arm_service(False)
                self._call_tasks_arm_service_callback(task_command[1], arm_result)
            elif(task_command[0] == 'nop'):
                self._request_zero_velocity()
            elif(task_command[0] == 'exit'):
                self._request_zero_velocity()
                return
            else:
                rospy.logerr('Unkown command request: %s, noping', task_command[0])
                self._request_zero_velocity()
            rate.sleep()

    def _request_zero_velocity(self):
        velocity = TwistStamped()
        velocity.header.frame_id = 'level_quad'
        velocity.header.stamp = rospy.Time.now()
        self._publish_twist(velocity)

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

    def _use_arm_service(self, arm):
        try:
            armed = self._arm_service(arm)
        except rospy.ServiceException as exc:
            rospy.logerr("Could not arm: " + str(exc))
            armed = False
        return armed

    def _publish_twist(self, twist):
        velocity_msg = TwistStampedArrayStamped()
        velocity_msg.header.stamp = rospy.Time.now()
        velocity_msg.data = [twist]
        self._velocity_pub.publish(velocity_msg)

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
                    return ('nop',)

            try:
                task_request = self._task.get_desired_command()
            except Exception as e:
                rospy.logerr('Exception getting tasks preferred velocity')
                rospy.logerr(str(e))
                rospy.logerr(traceback.format_exc())
                rospy.logwarn('Motion planner aborted task')
                self._action_server.set_aborted()
                self._task = None
                return ('nop',)
            
            task_state = task_request[0]
            if task_state == TaskState.canceled:
                self._action_server.set_canceled()
                self._task = None
            elif task_state == TaskState.aborted:
                rospy.logwarn('Task aborted with: %s', task_request[1])
                self._action_server.set_aborted()
                self._task = None
            elif task_state == TaskState.failed:
                rospy.logwarn('Task failed with: %s', task_request[1])
                self._action_server.set_succeeded(False)
                self._task = None
            elif task_state == TaskState.done:
                self._action_server.set_succeeded(True)
                self._task = None
            else:
                assert task_state == TaskState.running
                return task_request[1:]
        # No action to take return a nop
        return ('nop',)

if __name__ == '__main__':
    rospy.init_node('motion_planner')
    action_server = IarcTaskActionServer()

    motion_planner = MotionPlanner(action_server)
    try:
        motion_planner.run()
    except Exception, e:
        rospy.logfatal("Error in motion planner while running.")
        rospy.logfatal(str(e))
        raise
    finally:
        rospy.signal_shutdown("Motion Planner shutdown")

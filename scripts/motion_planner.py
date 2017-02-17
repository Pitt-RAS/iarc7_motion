#!/usr/bin/env python
import sys
import traceback
import rospy
from geometry_msgs.msg import TwistStamped
from iarc7_msgs.msg import TwistStampedArrayStamped
from std_srvs.srv import SetBool
from iarc_task_action_server import IarcTaskActionServer
from task_state import TaskState

class MotionPlanner:

    def __init__(self, _action_server):
        self._action_server = _action_server
        self.task = None
        self._velocity_pub = rospy.Publisher('movement_velocity_targets', TwistStampedArrayStamped, queue_size=0)
        self._arm_service = rospy.ServiceProxy('uav_arm', SetBool)

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
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
            self.task = None

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

    def _get_task_command(self):
        if (self.task is None) and self._action_server.has_new_task():
            self.task = self._action_server.get_new_task()

        if self.task:
            if self._action_server.is_canceled():
                try:
                    self.task.cancel()
                except Exception as e:
                    rospy.logerr('Exception canceling task')
                    rospy.logerr(str(e))
                    rospy.logerr(traceback.format_exc())
                    rospy.logwarn('Motion planner aborted task')
                    self._action_server.set_aborted()
                    self.task = None
                    return ('nop',)

            try:
                task_request = self.task.get_desired_command()
            except Exception as e:
                rospy.logerr('Exception getting tasks preferred velocity')
                rospy.logerr(str(e))
                rospy.logerr(traceback.format_exc())
                rospy.logwarn('Motion planner aborted task')
                self._action_server.set_aborted()
                self.task = None
                return ('nop',)
            
            task_state = task_request[0]
            if task_state == TaskState.canceled:
                self._action_server.set_canceled()
                self.task = None
            elif task_state == TaskState.aborted:
                rospy.logwarn('Task aborted with: %s', task_request[1])
                self._action_server.set_aborted()
                self.task = None
            elif task_state == TaskState.failed:
                rospy.logwarn('Task failed with: %s', task_request[1])
                self._action_server.set_succeeded(False)
                self.task = None
            elif task_state == TaskState.done:
                self._action_server.set_succeeded(True)
                self.task = None
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

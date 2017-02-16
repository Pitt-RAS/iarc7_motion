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
        self.velocity_pub = rospy.Publisher('movement_velocity_targets', TwistStampedArrayStamped, queue_size=0)
        self.arm_service = rospy.ServiceProxy('uav_arm', SetBool)

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            # Tuples could have different lengths so just get the whole tuple
            task_command = self.get_task_command()
            if(task_command[0] == 'velocity'):
                self.publish_twist(task_command[1])
            elif(task_command[0] == 'arm'):
                armed = False
                try:
                    armed = self.arm_service(True)
                except rospy.ServiceException as exc:
                    print("Could not arm: " + str(exc))
                task_command[1](armed)
            elif(task_command[0] == 'nop'):
                self.publish_twist(TwistStamped())
            else:
                rospy.logerr('Unkown command request: %s, noping', task_command[0])
                self.publish_twist(TwistStamped())
            rate.sleep()

    def publish_twist(self, twist):
        velocity_msg = TwistStampedArrayStamped()
        velocity_msg.header.stamp = rospy.Time.now()
        velocity_msg.data = [twist]
        self.velocity_pub.publish(velocity_msg)

    def get_task_command(self):
        if (self.task is None) and self._action_server.has_new_task():
            self.task = self._action_server.get_new_task()

        if self.task:
            if self._action_server.is_canceled():
                self.task.cancel()

            try:
                task_request = self.task.get_preferred_velocity()
            except Exception as e:
                rospy.logerr('Exception getting tasks preferred velocity')
                rospy.logerr(str(e))
                rospy.logerr(traceback.format_exc())
                self._action_server.set_aborted()
                self.task = None
                return ('nop',)
            
            task_state = task_request[0]
            if task_state == TaskState.canceled:
                self._action_server.set_canceled()
                self.task = None
            elif task_state == TaskState.aborted:
                rospy.logwarn(task_request[1])
                self._action_server.set_aborted()
                self.task = None
            elif task_state == TaskState.failed:
                rospy.logwarn(task_request[1])
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
        rospy.logfatal("Error in motion planner get velocity command.")
        rospy.logfatal(str(e))
        rospy.signal_shutdown("Motion Planner shutdown")
        raise

    rospy.signal_shutdown("Motion Planner shutdown")

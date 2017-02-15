#!/usr/bin/env python
import sys
import rospy
from geometry_msgs.msg import TwistStamped
from iarc7_msgs.msg import TwistStampedArrayStamped
from iarc_task_action_server import IarcTaskActionServer
from task_state import TaskState
import traceback

class MotionPlanner:

    def __init__(self, _action_server):
        self._action_server = _action_server
        self.task = None
        self.velocity_pub = rospy.Publisher('movement_velocity_targets', TwistStampedArrayStamped, queue_size=0)

    def get_velocity_command(self):
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
                return
            
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
                if(task_request[1] == 'velocity'):
                    velocity = task_request[2]
                    rospy.logwarn(str(velocity))
                    velocity_msg = TwistStampedArrayStamped()
                    velocity_msg.header.stamp = rospy.Time.now()
                    velocity_msg.data = [velocity]
                    self.velocity_pub.publish(velocity_msg)
        else:
            velocity_msg = TwistStampedArrayStamped()
            velocity_msg.header.stamp = rospy.Time.now()
            velocity_msg.data = [TwistStamped()]
            self.velocity_pub.publish(velocity_msg)

if __name__ == '__main__':
    rospy.init_node('motion_planner')
    action_server = IarcTaskActionServer()

    motion_planner = MotionPlanner(action_server)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        try:
            motion_planner.get_velocity_command()
        except Exception, e:
            rospy.logfatal("Error in motion planner get velocity command.")
            rospy.logfatal(str(e))
            rospy.signal_shutdown("Motion Planner shutdown")
            raise
        rate.sleep()

    rospy.signal_shutdown("Motion Planner shutdown")

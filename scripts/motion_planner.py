#!/usr/bin/env python
import sys
import rospy
from iarc_task_action_server import IarcTaskActionServer
from task_state import TaskState

class MotionPlanner:

    def __init__(self, action_server):
        self.action_server = action_server
        self.task = None

    def get_velocity_command(self):
        if (self.task is None) and self.action_server.has_new_task():
            self.task = self.action_server.get_new_task()

        if self.task:
            if self.action_server.is_canceled():
                print("canceling")
                self.task.cancel()

            (task_state, preferred_twist) = self.task.get_preferred_velocity()

            if task_state == TaskState.canceled:
                self.action_server.set_canceled()
                self.task = None
            elif task_state == TaskState.aborted:
                self.action_server.set_aborted()
                self.task = None
            elif task_state == TaskState.failed:
                self.action_server.set_succeeded(False)
                self.task = None
            elif task_state == TaskState.done:
                self.action_server.set_succeeded(True)
                self.task = None
            else:
                assert task_state == TaskState.running

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

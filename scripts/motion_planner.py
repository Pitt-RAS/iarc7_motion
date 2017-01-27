#!/usr/bin/env python
import sys
import rospy
from iarc_task_action_server import IarcTaskActionServer

class MotionPlanner:

    def __init__(self, action_server):
        self.action_server = action_server
        self.task = None

    def get_velocity_command(self):
        if (self.task is None) and self.action_server.has_new_task():
            self.task = self.action_server.get_new_task()

        if self.task:
            if self.action_server.is_canceled():
                self.task.cancel()
                self.action_server.set_canceled()
                self.task = None

            elif self.task.is_done():
                self.action_server.set_succeeded(self.task.get_result())
                self.task = None

            elif self.task.is_aborted():
                self.action_server.set_aborted()
                self.task = None

            else:
                preferred_twist = self.task.get_preferred_velocity()

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

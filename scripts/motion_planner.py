#!/usr/bin/env python
import rospy

class MotionPlanner:

    def __init__(self, action_server):
        self.action_server = action_server

        self.task = None

    def update():
        if (self.task is None) and self.action_server.has_new_task():
            self.task = self.action_server.get_new_task()

        if self.task:
            if self.action_server.is_canceled()
                self.task.cancel()
                self.action_server.set_canceled()

            self.task.update()

            if self.task.done():
                self.action_server.set_succeeded()
            elif self.task.aborted():
                self.action_server.set_aborted()
            else:
                preferred_twist = self.task.get_preferred_velocity()


    def preempt_task():
        print("Preempt")

    def append_task():
        print("Append")

    def get_velocity_command():
        print("Get velocity command")

if __name__ == '__main__':
    rospy.init_node('motion_planner', anonymous=True)

    action_server = ActionHandler()

    motion_planner = MotionPlanner(action_server)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        motion_planner.update()
        rate.sleep()

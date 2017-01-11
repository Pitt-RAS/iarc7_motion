#!/usr/bin/env python
import rospy

class MotionPlanner:

    def preempt_task():
        print("Preempt")

    def append_task():
        print("Append")

    def get_velocity_command():
        print("Get velocity command")

if __name__ == '__main__':
    rospy.init_node('motion_planner', anonymous=True)
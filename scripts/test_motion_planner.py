#! /usr/bin/env python
from __future__ import print_function
import sys
import rospy

import actionlib
from iarc7_motion.msg import QuadMoveGoal, QuadMoveAction

def motion_planner_client():
    # Creates the SimpleActionClient, passing the type of the action
    # (QuadMoveAction) to the constructor. (Look in the action folder)
    client = actionlib.SimpleActionClient("motion_planner_server", QuadMoveAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Test takeoff
    goal = QuadMoveGoal(movement_type="takeoff", takeoff_height=3.0)
    # Sends the goal to the action server.
    client.send_goal(goal)
    # Waits for the server to finish performing the action.
    client.wait_for_result()
    rospy.logwarn("Takeoff success: {}".format(client.get_result()))

    # Test takeoff
    goal = QuadMoveGoal(movement_type="land")
    # Sends the goal to the action server.
    client.send_goal(goal)
    # Waits for the server to finish performing the action.
    client.wait_for_result()
    rospy.logwarn("Land success: {}".format(client.get_result()))

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('test_motion_planner_client_py')
        motion_planner_client()
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
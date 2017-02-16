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

    # Test standard send receive
    # Creates a goal to send to the action server.
    goal = QuadMoveGoal(movement_type="test_task")
    # Sends the goal to the action server.
    client.send_goal(goal)
    # Waits for the server to finish performing the action.
    client.wait_for_result()
    rospy.loginfo("Request/wait for request success: {}".format(client.get_result()))

    # Test canceling goal
    client.send_goal(goal)
    rospy.sleep(0.5)
    client.cancel_goal()
    rospy.loginfo("Goal canceled")

    # Test aborting goal
    # Creates a goal to send to the action server, uses takeoff_height as a flag
    goal = QuadMoveGoal(movement_type="test_task", takeoff_height=1.0)
    client.send_goal(goal)
    client.wait_for_result()
    rospy.loginfo("Task aborted result (false is expected): {}".format(client.get_result()))

    rospy.loginfo("Attempting a cancel on a queued operation")
    rospy.sleep(3.0)

    # Test queueing then canceling a queued goal
    # Creates a goal to send to the action server.
    goal = QuadMoveGoal(movement_type="test_task")
    # This is improper usage because the simple action client only supports one active goal at a time
    # But iarc action server will queue multiple requests. By sending multiple goals,  the simple action
    # client loses the ability to stop or abort the first goal
    client.send_goal(goal)
    client.send_goal(goal)
    rospy.sleep(0.5)
    client.cancel_goal()
    
    # We should call wait_for_result but because the simple action client thinks the task is done
    # it immediately returns, sleep instead
    #client.wait_for_result()
    rospy.loginfo("Attempting a preempt operation")
    rospy.sleep(3.0)

    # Test preempting
    # This is also improper usage....
    goal = QuadMoveGoal(movement_type="test_task")
    client.send_goal(goal)
    client.send_goal(goal)
    rospy.sleep(0.5)
    goal = QuadMoveGoal(movement_type="test_task", preempt=True)
    client.send_goal(goal)
    client.wait_for_result()

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('test_motion_planner_client_py')
        motion_planner_client()
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
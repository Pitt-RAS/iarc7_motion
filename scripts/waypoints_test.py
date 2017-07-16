#!/usr/bin/env python
import rospy
import actionlib
import tf

import math

from actionlib_msgs.msg import GoalStatus

from iarc7_motion.msg import GroundInteractionGoal, GroundInteractionAction
from iarc7_msgs.msg import TwistStampedArray
from iarc7_msgs.srv import Arm
from geometry_msgs.msg import TwistStamped
from iarc7_safety.SafetyClient import SafetyClient

def constrain(x, l, h):
    return min(h, max(x, l))

if __name__ == '__main__':
    rospy.init_node('waypoints_test')

    safety_client = SafetyClient('motion_planner')
    assert safety_client.form_bond()

    velocity_pub = rospy.Publisher('movement_velocity_targets', TwistStampedArray, queue_size=0)
    tf_listener = tf.TransformListener()

    while not rospy.is_shutdown() and rospy.Time.now() == 0:
        pass
    start_time = rospy.Time.now()

    arm_service = rospy.ServiceProxy('uav_arm', Arm)
    arm_service.wait_for_service()
    armed = False
    while not rospy.is_shutdown() and not armed:
        try:
            armed = arm_service(True, True, True)
        except rospy.ServiceException as exc:
            print("Could not arm: " + str(exc))

    # Target points in global (X, Y, Z) coordinates
    waypoints = [
            (0, 0, 3, 0 * math.pi),
            (3, 0, 3, 1.75 * math.pi),
            (-3, 1, 4, 0.25 * math.pi),
            (-3, 1, 4, 1.5 * math.pi),
            (0, 0, 5, 1 * math.pi),
            (1, 2, 2, 1.25 * math.pi),
            ]
    waypoints_iter = iter(waypoints)
    target = next(waypoints_iter)

    kP = 0.5
    kP_yaw = 0.5
    max_vel = 1
    max_yaw_vel = 2.0 * math.pi / 3 # Max requested yaw is one rev per 3 seconds

    # Creates the SimpleActionClient for requesting ground interaction
    ground_interaction_client = actionlib.SimpleActionClient(
                                'ground_interaction_action',
                                GroundInteractionAction)
    ground_interaction_client.wait_for_server()

    # Request ground interaction of llm
    goal = GroundInteractionGoal(interaction_type='takeoff')
    # Sends the goal to the action server.
    ground_interaction_client.send_goal(goal)

    # Waits for the server to finish performing the action.
    ground_interaction_client.wait_for_result()
    rospy.logwarn("Takeoff success: {}".format(ground_interaction_client.get_result()))

    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        try:
            (trans, rot) = tf_listener.lookupTransform('/map', '/quad', rospy.Time(0))
        except tf.Exception as ex:
            rospy.logerr(ex.message)
            rate.sleep()
            continue

        # Exit immediately if fatal
        if safety_client.is_fatal_active():
            break;

        # Land if put into safety mode
        if safety_client.is_safety_active():
            target = (trans[0], trans[1], 0, 0)

        velocity = TwistStamped()
        velocity.header.frame_id = 'level_quad'
        velocity.header.stamp = rospy.Time.now()
        if abs(target[0] - trans[0]) >= 0.02:
            velocity.twist.linear.x = constrain((target[0] - trans[0]) * kP, -max_vel, max_vel)
        if abs(target[1] - trans[1]) >= 0.02:
            velocity.twist.linear.y = constrain((target[1] - trans[1]) * kP, -max_vel, max_vel)
        if abs(target[2] - trans[2]) >= 0.02:
            velocity.twist.linear.z = target[2] - trans[2]
        
        # Get the yaw (z axis) rotation from the quanternion
        current_yaw = tf.transformations.euler_from_quaternion(rot, 'rzyx')[0]
        
        # Transform current yaw to be between 0 and 2pi because the points are encoded from 0 to 2pi
        if current_yaw < 0:
            current_yaw = (2.0 * math.pi) + current_yaw

        # Calculate the difference
        yaw_difference = target[3] - current_yaw

        # Avoid taking the long way around
        if(yaw_difference > math.pi):
            yaw_difference = yaw_difference - 2.0 * math.pi

        # Avoid taking the long way around
        if(yaw_difference < -math.pi):
            yaw_difference = yaw_difference + 2.0 * math.pi

        # Finally set the desired twist velocity
        if abs(yaw_difference) >= 0.02:
            velocity.twist.angular.z = constrain(yaw_difference * kP_yaw, -max_yaw_vel, max_yaw_vel)

        velocity_msg = TwistStampedArray()
        velocity_msg.twists = [velocity]
        velocity_pub.publish(velocity_msg)

        if math.sqrt(sum((target[i] - trans[i])**2 for i in range(3))) < 0.1:
            if abs(yaw_difference) < 0.15:
                target = next(waypoints_iter, target)

        rate.sleep()

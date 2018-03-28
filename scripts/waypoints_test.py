#!/usr/bin/env python
import rospy
import actionlib
import tf

import math

from actionlib_msgs.msg import GoalStatus

from iarc7_motion.msg import GroundInteractionGoal, GroundInteractionAction
from iarc7_msgs.msg import MotionPointStampedArray, MotionPointStamped
from iarc7_msgs.srv import Arm
from iarc7_safety.SafetyClient import SafetyClient

def constrain(x, l, h):
    return min(h, max(x, l))

if __name__ == '__main__':
    rospy.init_node('waypoints_test')

    safety_client = SafetyClient('motion_command_coordinator')
    assert safety_client.form_bond()

    motion_point_pub = rospy.Publisher('motion_point_targets', MotionPointStampedArray, queue_size=0)
    tf_listener = tf.TransformListener()

    while not rospy.is_shutdown() and rospy.Time.now() == 0:
        pass
    start_time = rospy.Time.now()

    arm_service = rospy.ServiceProxy('uav_arm', Arm)
    arm_service.wait_for_service()
    armed = False
    while not rospy.is_shutdown() and not armed:
        try:
            armed = arm_service(True)
        except rospy.ServiceException as exc:
            print("Could not arm: " + str(exc))

    # Target points in global (X, Y, Z, Yaw) coordinates
    waypoints = [
            (0, 0, .3, 0 * math.pi),
            (0, 0, .4, 0 * math.pi),
            (0, 0, .5, 0 * math.pi),
            (0, 0, .7, 1.75 * math.pi),
            (0, 0, .9, 0.25 * math.pi),
            (0, 0, .9, 1.5 * math.pi),
            (0, 0, .9, 1 * math.pi),
            (0, 0, .9, 1 * math.pi),
            (0, 0, .5, 1 * math.pi),
            (0, 0, .9, 1 * math.pi),
            (0, 0, .5, 1 * math.pi),
            ]
    waypoints_iter = iter(waypoints)
    target = waypoints_iter.next()

    kP = 0.8
    gamma = 0.8
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
    time = rospy.Time.now().to_sec()
    while not rospy.is_shutdown():
        try:
            (trans, rot) = tf_listener.lookupTransform('/map',
                                                       '/quad',
                                                       rospy.Time(0))
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

        motion_point = MotionPointStamped()
        motion_point.header.frame_id = 'level_quad'
        motion_point.header.stamp = rospy.Time.now()

        #Setting x,y velocities and z position
        if abs(target[0] - trans[0]) >= 0.02:
            error = target[0] - trans[0]
            target_v = math.copysign(kP * abs(error)**gamma, error)
            motion_point.motion_point.twist.linear.x = constrain(target_v, -max_vel, max_vel)
        
        if abs(target[1] - trans[1]) >= 0.02:
            error = target[1] - trans[1]
            target_v = math.copysign(kP * abs(error)**gamma, error)
            motion_point.motion_point.twist.linear.y = constrain(target_v, -max_vel, max_vel)
        
        motion_point.motion_point.pose.position.z = target[2]

        motion_point_msg = MotionPointStampedArray()
        motion_point_msg.motion_points = [motion_point]
        motion_point_pub.publish(motion_point_msg)

        #Sets next target after a set time has passed
        if ((rospy.Time.now().to_sec() - time) > 1):
            try:
                target = waypoints_iter.next()
            except StopIteration:
                break
            time = rospy.Time.now().to_sec()
            rospy.logerr('Switching targets')
            print(target)

        rate.sleep()

    # Test land
    goal = GroundInteractionGoal(interaction_type = "land")
    # Sends the goal to the action server.
    ground_interaction_client.send_goal(goal)
    # Waits for the server to finish performing the action.
    ground_interaction_client.wait_for_result()
    rospy.logwarn("Land success: {}".format(ground_interaction_client.get_result()))
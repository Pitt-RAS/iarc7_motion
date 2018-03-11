#!/usr/bin/env python
import rospy

from iarc7_msgs.msg import TwistStampedArray
from iarc7_msgs.msg import UAVMotionPointStampedArray, UAVMotionPointStamped

def callback(msg):
    motion_points = UAVMotionPointStampedArray()

    for twist in msg.twists:
        motion_point = UAVMotionPointStamped()
        motion_point.header = twist.header
        motion_point.motion_point.twist = twist.twist
        motion_points.motion_points.append(motion_point)

    motion_point_pub.publish(motion_points)


if __name__ == '__main__':
    rospy.init_node('thrust_target_to_motion_point_transcriber')

    velocity_sub = rospy.Subscriber('movement_velocity_targets', TwistStampedArray, callback)
    motion_point_pub = rospy.Publisher('uav_motion_point_targets', UAVMotionPointStampedArray, queue_size=0)

    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        rate.sleep()

#!/usr/bin/env python
import rospy

from iarc7_msgs.msg import TwistStampedArray
from iarc7_msgs.msg import MotionPointStampedArray, MotionPointStamped

def callback(msg):
    motion_points = MotionPointStampedArray()

    for twist in msg.twists:
        motion_point = MotionPointStamped()
        motion_point.header = twist.header
        motion_point.motion_point.twist = twist.twist
        motion_points.motion_points.append(motion_point)

    motion_point_pub.publish(motion_points)


if __name__ == '__main__':
    rospy.init_node('thrust_target_to_motion_point_transcriber')

    velocity_sub = rospy.Subscriber('movement_velocity_targets', TwistStampedArray, callback)
    motion_point_pub = rospy.Publisher('motion_point_targets', MotionPointStampedArray, queue_size=0)

    rospy.spin()

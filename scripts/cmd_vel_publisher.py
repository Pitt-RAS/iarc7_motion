#!/usr/bin/env python

import rospy

from geometry_msgs.msg import TwistStamped
from iarc7_msgs.msg import TwistStampedArrayStamped

def velocity_callback(msg):
    if msg.data:
        pub.publish(msg.data[0])

if __name__ == '__main__':
    rospy.init_node('cmd_vel_publisher')
    rospy.Subscriber('movement_velocity_targets', TwistStampedArrayStamped, velocity_callback)
    pub = rospy.Publisher('cmd_vel', TwistStamped, queue_size=0)
    rospy.spin()

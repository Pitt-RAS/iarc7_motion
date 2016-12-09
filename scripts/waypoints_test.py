#!/usr/bin/env python
import rospy
import tf

import math

from iarc7_msgs.msg import TwistStampedArrayStamped
from geometry_msgs.msg import TwistStamped

def constrain(x, l, h):
    return min(h, max(x, l))

if __name__ == '__main__':
    rospy.init_node('waypoints_test', anonymous=True)

    velocity_pub = rospy.Publisher('movement_velocity_targets', TwistStampedArrayStamped, queue_size=0)
    tf_listener = tf.TransformListener()

    while rospy.Time.now() == 0:
        pass
    start_time = rospy.Time.now()

    # Target points in global (X, Y, Z) coordinates
    waypoints = [
            (0, 0, 3),
            (3, 0, 3),
            (-3, 1, 4),
            ]
    waypoints_iter = iter(waypoints)
    target = next(waypoints_iter)

    kP = 0.5
    max_vel = 1

    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        try:
            (trans, rot) = tf_listener.lookupTransform('/map', '/level_quad', rospy.Time(0))
        except tf.Exception as ex:
            rospy.logerr(ex.message)
            continue

        velocity = TwistStamped()
        velocity.header.frame_id = 'level_quad'
        velocity.header.stamp = rospy.Time.now() + rospy.Duration(1)
        if abs(target[0] - trans[0]) >= 0.02:
            velocity.twist.linear.x = constrain((target[0] - trans[0]) * kP, -max_vel, max_vel)
        if abs(target[1] - trans[1]) >= 0.02:
            velocity.twist.linear.y = constrain((target[1] - trans[1]) * kP, -max_vel, max_vel)
        if abs(target[2] - trans[2]) >= 0.02:
            velocity.twist.linear.z = target[2] - trans[2]
        print velocity

        velocity_msg = TwistStampedArrayStamped()
        velocity_msg.header.stamp = rospy.Time.now()
        velocity_msg.data = [velocity]
        velocity_pub.publish(velocity_msg)

        if math.sqrt(sum((target[i] - trans[i])**2 for i in range(3))) < 0.1:
            target = next(waypoints_iter, target)

        rate.sleep()

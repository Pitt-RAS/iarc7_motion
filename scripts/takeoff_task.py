#!/usr/bin/env python
import rospy
from abstract_task import AbstractTask
from task_state import TaskState
from geometry_msgs.msg import TwistStamped

import rospy
import tf

import math

from iarc7_msgs.msg import TwistStampedArrayStamped
from geometry_msgs.msg import TwistStamped
from iarc7_safety.SafetyClient import SafetyClient


class TakeoffTask(AbstractTask):

    def __init__(self, actionvalues_dict):
        self.takeoff_height = actionvalues_dict['takeoff_height']
        #self.velocity_pub = rospy.Publisher('movement_velocity_targets', TwistStampedArrayStamped, queue_size=0)
        self.tf_listener = tf.TransformListener()
        self.canceled = False;

    def get_preferred_velocity(self):
        if self.canceled:
            return (TaskState.canceled, TwistStamped());

        while rospy.Time.now() == 0:
            pass

        kP = 0.5
        kP_yaw = 0.5
        max_vel = 1
        max_yaw_vel = 2.0 * math.pi / 3 # Max requested yaw is one rev per 3 seconds

        try:
            (trans, rot) = tf_listener.lookupTransform('/map', '/quad', rospy.Time(0))
        except tf.Exception as ex:
            rospy.logerr(ex.message)
            rate.sleep()
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

        velocity_msg = TwistStampedArrayStamped()
        velocity_msg.header.stamp = rospy.Time.now()
        velocity_msg.data = [velocity]
        velocity_pub.publish(velocity_msg)
        rospy.loginfo("TakeoffTask get_preferred_velocity")
        
        if math.sqrt(sum((target[i] - trans[i])**2 for i in range(3))) < 0.1:
            if abs(yaw_difference) < 0.15:
                return (TaskState.done, TwistStamped())

        return (TaskState.running, velocity_msg)

    def cancel(self):
        rospy.loginfo("TakeoffTask canceled")
        self.canceled = True

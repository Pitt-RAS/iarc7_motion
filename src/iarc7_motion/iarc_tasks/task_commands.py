#!/usr/bin/env python

import rospy

from geometry_msgs.msg import TwistStamped

class VelocityCommand(object):
    def __init__(self,  target_twist=None):
        if target_twist is None:
            self.target_twist = TwistStamped()
            self.target_twist.header.stamp = rospy.Time.now()
        else:
            self.target_twist = target_twist

class ArmCommand(object):
    def __init__(self, arm_state, completion_callback):
        self.arm_state = arm_state
        self.completion_callback = completion_callback

class NopCommand(object):
    def __init__(self):
        pass

class GroundInteractionCommand(object):
    def __init__(self, interaction_type, completion_callback):
        self.interaction_type = interaction_type
        self.completion_callback = completion_callback

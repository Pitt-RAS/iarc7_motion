#!/usr/bin/env python

import rospy

from geometry_msgs.msg import TwistStamped, Vector3

class VelocityCommand(object):
    def __init__(self,
                 target_twist=None,
                 start_position_x=None,
                 start_position_y=None,
                 start_position_z=None,
                 start_velocity_x=None,
                 start_velocity_y=None,
                 start_velocity_z=None,
                 acceleration=None):

        self.start_position = Vector3()
        self.start_position.x = start_position_x
        self.start_position.y = start_position_y
        self.start_position.z = start_position_z

        self.start_velocity = Vector3()
        self.start_velocity.x = start_velocity_x
        self.start_velocity.y = start_velocity_y
        self.start_velocity.z = start_velocity_z

        self.acceleration = acceleration

        if target_twist is None:
            self.target_twist = TwistStamped()
            self.target_twist.header.stamp = rospy.Time.now()
        else:
            self.target_twist = target_twist

class GlobalPlanCommand(object):
    def __init__(self, plan):
        self.plan = plan

class ResetLinearProfileCommand(object):
    def __init__(self,
                 target_twist=None,
                 start_position_x=None,
                 start_position_y=None,
                 start_position_z=None,
                 start_velocity_x=None,
                 start_velocity_y=None,
                 start_velocity_z=None):

        self.start_position = Vector3()
        self.start_position.x = start_position_x
        self.start_position.y = start_position_y
        self.start_position.z = start_position_z

        self.start_velocity = Vector3()
        self.start_velocity.x = start_velocity_x
        self.start_velocity.y = start_velocity_y
        self.start_velocity.z = start_velocity_z

class NopCommand(object):
    def __init__(self):
        pass

class GroundInteractionCommand(object):
    def __init__(self, interaction_type, completion_callback):
        self.interaction_type = interaction_type
        self.completion_callback = completion_callback

class ConfigurePassthroughMode(object):
    def __init__(self, enable, completion_callback):
        self.enable = enable
        self.completion_callback = completion_callback

class AngleThrottleCommand(object):
    def __init__(self, pitch, roll, vyaw, vz):
        self.pitch = pitch
        self.roll = roll
        self.vyaw = vyaw
        self.vz = vz

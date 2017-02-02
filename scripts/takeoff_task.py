#!/usr/bin/env python
import rospy
from abstract_task import AbstractTask
from task_state import TaskState
from geometry_msgs.msg import TwistStamped

class TakeoffTask(AbstractTask):

    def __init__(self, takeoff_height):
        self.takeoff_height = takeoff_height

    def get_preferred_velocity(self):
        rospy.loginfo("TakeoffTask get_preferred_velocity")
        return (TaskState.done, TwistStamped())

    def cancel(self):
        rospy.loginfo("TakeoffTask canceled")

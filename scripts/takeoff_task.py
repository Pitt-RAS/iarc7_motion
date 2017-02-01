#!/usr/bin/env python
import rospy
from abstract_task import AbstractTask
from task_state import TaskState

class TakeoffTask(AbstractTask):

    def __init__(self, takeoff_height):
        self.takeoff_height = takeoff_height

    def get_preferred_velocity(self):
        rospy.loginfo("TakeoffTask get_preferred_velocity")
        return (TaskState.done, 0.0)

    def cancel(self):
        rospy.loginfo("TakeoffTask canceled")

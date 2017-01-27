#!/usr/bin/env python
import rospy
from abstract_task import AbstractTask

class TakeoffTask(AbstractTask):

    def __init__(self, takeoff_height):
        self.takeoff_height = takeoff_height

    def get_preferred_velocity(self):
        rospy.loginfo("get_preferred_velocity")
        return 5

    def cancel(self):
        return True

    def is_done(self):
        return True

    def is_aborted(self):
        return False

    def get_result(self):
        return False

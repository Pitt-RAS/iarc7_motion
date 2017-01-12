#!/usr/bin/env python
import rospy
from abstract_task import AbstractTask

class TakeoffTask(AbstractTask):

    def __init_(self, goal):
        self.goal = goal

    # Abstract method
    def get_preferred_velocity(self):
        rospy.logdebug("get_preferred_velocity")
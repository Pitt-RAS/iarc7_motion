#!/usr/bin/env python
import rospy
from abstract_task import AbstractTask

class TestTask(AbstractTask):

    def __init__(self, abort):
        self.target = None
        self.abort_time = None
        self.success = False
        self.abort = abort
        self.aborted = False
        self.started = False

    def get_preferred_velocity(self):
        if self.target is None:
            self.target = rospy.Time.now() + rospy.Duration(1.5)
            self.abort_time = rospy.Time.now() + rospy.Duration(0.75)

        if self.abort > 0.5:
            if self.abort_time < rospy.Time.now():
                self.aborted = True
        return self.target - rospy.Time.now()

    def cancel(self):
        rospy.loginfo("TestTask canceled")
        return True

    def is_done(self):
        if self.target is not None:
            if self.target  < rospy.Time.now():
                rospy.loginfo("TestTask done")
                self.success = True
                return True
            else:
                rospy.loginfo("TestTask not done")
                return False
        # Haven't been initialized yet
        return False

    def is_aborted(self):
        if self.aborted:
            rospy.loginfo("TestTask aborted")
        return self.aborted

    def get_result(self):
        return self.success

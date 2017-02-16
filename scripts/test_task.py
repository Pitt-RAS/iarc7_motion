#!/usr/bin/env python
import rospy
from abstract_task import AbstractTask
from task_state import TaskState
from geometry_msgs.msg import TwistStamped

class TestTask(AbstractTask):

    def __init__(self, abort):
        self.target = None
        self.abort_time = None
        self.abort = abort
        self.aborted = False
        self.canceled = False

    def get_desired_command(self):
        if self.target is None:
            self.target = rospy.Time.now() + rospy.Duration(1.5)
            self.abort_time = rospy.Time.now() + rospy.Duration(0.75)

        result = self.target - rospy.Time.now()

        if self.abort > 0.5:
            if self.abort_time < rospy.Time.now():
                rospy.loginfo("TestTask aborted")
                return (TaskState.aborted, result)

        if self.canceled:
            rospy.loginfo("TestTask canceled")
            return (TaskState.canceled, result)

        if self.target  < rospy.Time.now():
            rospy.loginfo("TestTask done")
            return (TaskState.done, result)
        else:
            rospy.loginfo("TestTask not done")

        return (TaskState.running, 'nop')

    def cancel(self):
        rospy.loginfo("TestTask canceling")
        self.canceled = True

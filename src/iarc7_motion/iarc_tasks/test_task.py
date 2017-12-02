#!/usr/bin/env python
import rospy

from geometry_msgs.msg import TwistStamped

from .abstract_task import AbstractTask
from iarc_tasks.task_states import (TaskRunning,
                                    TaskDone,
                                    TaskCanceled,
                                    TaskAborted)
from iarc_tasks.task_commands import (VelocityCommand,
                                      NopCommand)

class TestTask(AbstractTask):

    def __init__(self, task_request):
        self._transition = None
        self.target = None
        self.abort_time = None
        self.abort = task_request.takeoff_height
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
                return (TaskAborted(), result)

        if self.canceled:
            rospy.loginfo("TestTask canceled")
            return (TaskCanceled(),)

        if self.target  < rospy.Time.now():
            rospy.loginfo("TestTask done")
            return (TaskDone(), result)
        else:
            rospy.loginfo("TestTask not done")

        return (TaskRunning(), NopCommand())

    def cancel(self):
        rospy.loginfo("TestTask canceling")
        self.canceled = True
    
    def set_incoming_transition(self, transition):
        self._transition = transition

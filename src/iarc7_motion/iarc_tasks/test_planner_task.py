#!/usr/bin/env python
import rospy
import actionlib

from .abstract_task import AbstractTask

from iarc_tasks.task_states import (TaskRunning,
                                    TaskDone,
                                    TaskCanceled,
                                    TaskAborted)
from iarc_tasks.task_commands import NopCommand

from iarc7_msgs.msg import PlanGoal, PlanAction, MotionPointStamped

class TestPlannerTask(AbstractTask):

    def __init__(self, task_request):
        super(TestPlannerTask, self).__init__()

        # Creates the SimpleActionClient for requesting ground interaction
        self._client = actionlib.SimpleActionClient('planner_request', PlanAction)
        self._client.wait_for_server()
        self._plan_canceled = False

    def get_desired_command(self):
        goal = PlanGoal()
        goal.goal.motion_point.pose.position.x = 5
        goal.goal.motion_point.pose.position.y = 5
        self._client.send_goal(goal, None, None, self._feedback_callback)
        if self._plan_canceled: 
            return (TaskDone(), NopCommand())
        else: 
            return (TaskRunning(),)

    def _feedback_callback(self, msg): 
        self._feedback = msg
        rospy.logwarn('Feedback recieved')
        self._client.cancel_goal()
        self._plan_canceled = True

    def cancel(self):
        rospy.loginfo("TestPlannerTask canceling")
        self.canceled = True
        return True
    
    def set_incoming_transition(self, transition):
        self._transition = transition

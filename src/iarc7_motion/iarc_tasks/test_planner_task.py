#!/usr/bin/env python
import rospy
import actionlib

from .abstract_task import AbstractTask

from iarc_tasks.task_states import (TaskRunning,
                                    TaskDone,
                                    TaskCanceled,
                                    TaskAborted)
from iarc_tasks.task_commands import NopCommand

from iarc7_planner.msg import PlanGoal, PlanAction

class TestPlannerTask(AbstractTask):

    def __init__(self, task_request):
        super(TestPlannerTask, self).__init__()

        # Creates the SimpleActionClient for requesting ground interaction
        self._client = actionlib.SimpleActionClient('planner_request', PlanAction)
        self._client.wait_for_server()

    def get_desired_command(self):
        goal = PlanGoal(frame_id='test_planner')
        self._client.send_goal(goal, None, None, self._feedback_callback)
        self._client.wait_for_result()
        rospy.logwarn("Planning success: {}".format(self._client.get_result().success))

        return (TaskDone(), NopCommand())

    def _feedback_callback(self, msg): 
        self._feedback = msg
        rospy.logwarn(msg.plan.header.frame_id)

    def cancel(self):
        rospy.loginfo("TestPlannerTask canceling")
        self.canceled = True
        return True
    
    def set_incoming_transition(self, transition):
        self._transition = transition

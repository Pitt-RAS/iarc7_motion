#!/usr/bin/env python
import rospy
from takeoff_task import TakeoffTask
from iarc7_motion import QuadMove

class ActionHandler:
    def __init__(self):
        self._action_name = "motionplannerserver"


        self._as = actionlib.ActionServer(self._action_name,
                                          QuadMove,
                                          execute_cb=self.new_goal,
                                          cancel_cb=self.cancel_request,
                                          auto_start = False)
        self._as.start()

        self.goal_tasks = []
        self.current_task = None
        self.current_goal = None
        self.cancel_requested = False

    def new_goal(self, goal):
        rospy.loginfo("new_goal: %s", goal.get_goal_id().id)

        movement_type = goal.get_goal()
        if movement_type == "takeoff" :
            goal_tasks.append([goal, TakeoffTask()])
        else:
            rospy.logerror("Goal has invalid movement_type: %s", movement_type)
            goal.set_rejected()

    def cancel_request(self, cancel):
        rospy.logdebug("cancel_request")

        if cancel == self.current_goal :
            rospy.logdebug("Cancel requested on current goal")
            self.current_goal.set_cancel_requested()
            self.cancel_requested = True
            return

        for goal_task in goal_tasks:
            if goal_task[0] == cancel:
                rospy.logdebug("Cancel requested on queued goal")
                goal_task[0].set_cancel_requested()
                goal_task[0].set_canceled()
                return

        rospy.logerror("Attempt to cancel goal but goal did not exist")

    # Function for task runner to use
    def set_succeeded(self):
        if self.current_goal:
            rospy.logdebug("Current task succeeded")
            self.current_goal.set_succeeded()
        else:
            rospy.logdebug("There was not task to succeed")

        self.current_task = None
        self.current_goal = None
        self.cancel_requested = False

    def set_aborted(self):
        if self.current_goal:
            rospy.logdebug("Current task aborted")
            self.current_goal.set_aborted()
        else:
            rospy.logdebug("There was not task to abort")

        self.current_task = None
        self.current_goal = None
        self.cancel_requested = False

    def set_canceled():
        if self.current_goal :
            rospy.logdebug("Current task cancelled")
            self.current_goal.set_canceled()
        else:
            rospy.logdebug("There was not a task to cancel")

        self.current_task = None
        self.current_goal = None
        self.cancel_requested = False

    def is_canceled(self):
        return self.cancel_requested

    def get_new_task(self):
        if len(goal_tasks) == 0:
            return None
        
        goal_task = goal_tasks.pop(0)
        self.current_goal = goal_task[0]
        self.current_task = goal_task[1]
        self.cancel_requested = False

        rospy.logdebug("New task accepted")
        self.current_goal.set_accepted()

        return self.current_task
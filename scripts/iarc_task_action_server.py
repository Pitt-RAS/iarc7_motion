#!/usr/bin/env python
import threading
import rospy
import actionlib
from takeoff_task import TakeoffTask
from test_task import TestTask
from iarc7_motion.msg import QuadMoveAction, QuadMoveResult

class IarcTaskActionServer:
    def __init__(self):
        self._action_name = "motion_planner_server"


        self._action_server = actionlib.ActionServer(self._action_name,
                                          QuadMoveAction,
                                          self._new_goal,
                                          cancel_cb=self._cancel_request,
                                          auto_start = False)
        self._goal_tasks = []
        self._current_task = None
        self._current_goal = None
        self._cancel_requested = False
        self._lock = threading.RLock()
        # Start action server last to avoid race condition
        self._action_server.start()

        self._task_dict = {'takeoff': TakeoffTask,
                           'test_task': TestTask}

    # Private method
    def _new_goal(self, goal):
        with self._lock:
            rospy.logdebug("_new_goal: %s", goal.get_goal_id().id)

            task_request = goal.get_goal()

            try:
                new_task = self._task_dict[task_request.movement_type](task_request.takeoff_height)
            except KeyError as e:
                rospy.logerr("Goal has invalid movement_type: %s", task_request.movement_type)
                goal.set_rejected()
                return

            # Support simple queue destroying preempting for now
            if task_request.preempt :
                if len(self._goal_tasks) > 0:
                    for x, _ in  self._goal_tasks:
                        x.set_cancel_requested()
                        x.set_canceled()
                    self._goal_tasks = []
                if self._current_goal:
                    self._cancel_requested = True
                    self._current_goal.set_cancel_requested()

            self._goal_tasks.append((goal, new_task))

    # Private method
    def _cancel_request(self, cancel):
        with self._lock:
            rospy.logdebug("cancel_request")

            if cancel == self._current_goal :
                rospy.logdebug("Cancel requested on current goal")
                self._current_goal.set_cancel_requested()
                self._cancel_requested = True
                return

            length = len(self._goal_tasks)
            self._goal_tasks[:] = [goal_task for goal_task in self._goal_tasks if not self._determine_canceled(goal_task, cancel)]

            if len(self._goal_tasks) + 1 == length:
                rospy.logdebug("Cancel requested on queued goal")
            else:
                rospy.logerr("Attempt to cancel goal but goal did not exist or more than one goal was deleted")

    # Private method
    def _determine_canceled(self, goal_task, cancel):
        goal = goal_task[0]
        if goal == cancel:
            goal.set_cancel_requested()
            goal.set_canceled()
            return True
        return False

    # Function for task runner to use
    def set_succeeded(self, success):
        with self._lock:
            if self._current_goal:
                rospy.logdebug("Current task succeeded")
                reponse = QuadMoveResult(success=success)
                self._current_goal.set_succeeded(reponse)
            else:
                rospy.logdebug("There was not task to succeed")

            self._current_task = None
            self._current_goal = None
            self._cancel_requested = False

    def set_aborted(self):
        with self._lock:
            if self._current_goal:
                rospy.logdebug("Current task aborted")
                response = QuadMoveResult(success=False)
                self._current_goal.set_aborted(result=response)
            else:
                rospy.logdebug("There was not task to abort")

            self._current_task = None
            self._current_goal = None
            self._cancel_requested = False

    def set_canceled(self):
        with self._lock:
            if self._current_goal :
                rospy.logdebug("Current task cancelled")
                self._current_goal.set_canceled()
            else:
                rospy.logdebug("There was not a task to cancel")

            self._current_task = None
            self._current_goal = None
            self._cancel_requested = False

    def is_canceled(self):
        with self._lock:
            return self._cancel_requested

    def get_new_task(self):
        with self._lock:
            if len(self._goal_tasks) == 0:
                return None
            
            self._current_goal, self._current_task= self._goal_tasks.pop(0)
            self._cancel_requested = False

            rospy.logdebug("New task accepted")
            self._current_goal.set_accepted()

            return self._current_task

    def has_new_task(self):
        with self._lock:
            return (len(self._goal_tasks) > 0)

import math
import rospy
import tf2_ros
import threading
import cPickle as pickle

from .abstract_task import AbstractTask
from iarc_tasks.task_states import (TaskRunning,
                                    TaskDone,
                                    TaskCanceled,
                                    TaskAborted,
                                    TaskFailed)
from iarc_tasks.task_commands import NopCommand, GlobalPlanCommand

from iarc7_msgs.msg import PlanGoal, PlanAction, MotionPointStamped

class XYZTranslationTaskState(object):
    INIT = 1
    WAITING = 2
    PLAN_RECEIVED = 3
    COMPLETING = 4

class XYZTranslationTask(AbstractTask):
    def __init__(self, task_request):
        super(XYZTranslationTask, self).__init__()

        self._canceled = False

        self._transition = None

        self._plan = None
        self._feedback = None

        self._complete_time = None
        self._sent_plan_time = None

        self._starting_motion_point = None

        # self._list_info = []
        # self._request_list = []

        self._lock = threading.RLock()

        self._linear_gen = self.topic_buffer.get_linear_motion_profile_generator()

        try:
            self._PLANNING_TIMEOUT = rospy.Duration(rospy.get_param('~planning_timeout'))
            self._PLANNING_LAG = rospy.Duration(rospy.get_param('~planning_lag'))
            self._DONE_REPLAN_DIST = rospy.get_param('~done_replanning_radius')
            self._MIN_MANEUVER_HEIGHT = rospy.get_param('~min_maneuver_height')
        except KeyError as e:
            rospy.logerr('Could not lookup a parameter for xyztranslation task')
            raise

        # Check that we aren't being requested to go below the minimum maneuver height
        # Error straight out if that's the case.
        if task_request.z_position < self._MIN_MANEUVER_HEIGHT:
            raise ValueError('Requested z height was below the minimum maneuver height')

        self._goal_x = task_request.x_position
        self._goal_y = task_request.y_position
        self._goal_z = task_request.z_position

        self._state = XYZTranslationTaskState.INIT

    def get_desired_command(self):
        with self._lock:
            if self._canceled:
                return (TaskCanceled(),)

            if self._state == XYZTranslationTaskState.COMPLETING:
                if (rospy.Time.now() + rospy.Duration(.10)) >= self._complete_time:
                    # name = '/home/andrew/data.bin'
                    # file = open(name, 'wb')
                    # pickle.dump(self._list_info, file)
                    # file.close()
                    # self._linear_gen.dump_info()
                    return (TaskDone(),)
                else:
                    return (TaskRunning(), NopCommand())

            expected_time = rospy.Time.now() + self._PLANNING_LAG
            self._starting_motion_point = self._linear_gen.expected_point_at_time(expected_time).motion_point

            _starting_pose = self._starting_motion_point.pose.position
            _distance_to_goal = math.sqrt(
                        (_starting_pose.x-self._goal_x)**2 +
                        (_starting_pose.y-self._goal_y)**2)

            if _distance_to_goal < self._DONE_REPLAN_DIST:
                if self._plan is not None:
                    try:
                        self._complete_time = self._plan.motion_points[-1].header.stamp
                    except:
                        rospy.logerr('Planner returned an empty plan while XYZ Translate was in COMPLETING state')
                        return (TaskAborted(),)
                    self._state = XYZTranslationTaskState.COMPLETING
                    return (TaskRunning(), GlobalPlanCommand(self._plan))
                else:
                    rospy.logerr('XYZTranslationTask: Plan is None but we are done')
                    return (TaskFailed(msg='Started too close to goal to do anything'),)

            if self._state == XYZTranslationTaskState.INIT:
                self.topic_buffer.make_plan_request(self._generate_request(expected_time),
                                                    self._feedback_callback)
                self._state = XYZTranslationTaskState.WAITING

            if self._state == XYZTranslationTaskState.PLAN_RECEIVED:
                if self._feedback is not None:
                    self.topic_buffer.make_plan_request(
                        self._generate_request(expected_time, self._feedback.success),
                        self._feedback_callback)

                    self._state = XYZTranslationTaskState.WAITING
                    if self._feedback.success:
                        # send LLM the plan we received
                        return (TaskRunning(), GlobalPlanCommand(self._plan))
                else:
                    rospy.logerr('XYZTranslationTask: In PLAN_RECEIVED state but no feedback')
                    return (TaskFailed(msg='In PLAN_RECEIVED state but no feedback'),)

            if (self._sent_plan_time is not None and
                (rospy.Time.now() - self._sent_plan_time) > self._PLANNING_TIMEOUT):
                return (TaskFailed(msg='XYZ Translate: planner took too long to plan'),)

            return (TaskRunning(), NopCommand())

    def _generate_request(self, expected_time, reset_timer=True):
        request = PlanGoal()
        request.header.stamp = expected_time

        start = MotionPointStamped()
        start.motion_point = self._starting_motion_point

        goal = MotionPointStamped()
        goal.motion_point.pose.position.x = self._goal_x
        goal.motion_point.pose.position.y = self._goal_y
        goal.motion_point.pose.position.z = self._goal_z

        # self._request_list = [start, goal, None]

        request.start = start
        request.goal = goal

        if reset_timer:
            self._sent_plan_time = rospy.Time.now()

        return request

    def _feedback_callback(self, status, msg):
        with self._lock:
            self._feedback = msg
            self._plan = msg.plan
            # self._request_list[2] = msg.plan
            # self._list_info.append(self._request_list)
            self._state = XYZTranslationTaskState.PLAN_RECEIVED

    def cancel(self):
        rospy.loginfo('XYZTranslationTask canceled')
        self._canceled = True
        return True

    def set_incoming_transition(self, transition):
        self._transition = transition

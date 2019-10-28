import math
import rospy
import tf2_ros
import threading
import time

from .abstract_task import AbstractTask
from iarc_tasks.task_states import (TaskRunning,
                                    TaskDone,
                                    TaskCanceled,
                                    TaskAborted,
                                    TaskFailed)
from iarc_tasks.task_commands import NopCommand, GlobalPlanCommand

from iarc7_msgs.msg import PlanGoal, PlanAction, MotionPointStamped


class TestPlannerTask(AbstractTask):
    def __init__(self, task_request):
        super(TestPlannerTask, self).__init__()

        self._canceled = False

        self._transition = None
        self._plan = None
        self._feedback = None
        self._feedback_receieved = False
        self._starting = True

        self._corrected_start_x = None
        self._corrected_start_y = None
        self._corrected_start_z = None

        self._vel_start = None

        self._lock = threading.RLock()

        self._linear_gen = self.topic_buffer.get_linear_motion_profile_generator()

        try:
            self._PLANNING_LAG = rospy.Duration(rospy.get_param('~planning_lag'))
            self._COORDINATE_FRAME_OFFSET = rospy.get_param('~planner_coordinate_frame_offset')
            self._MIN_MANEUVER_HEIGHT = rospy.get_param('~min_maneuver_height')
        except KeyError as e:
            rospy.logerr('Could not lookup a parameter for TestPlannerTask task')
            raise

        # Check that we aren't being requested to go below the minimum maneuver height
        # Error straight out if that's the case.
        if task_request.z_position < self._MIN_MANEUVER_HEIGHT :
            raise ValueError('Requested z height was below the minimum maneuver height')

        self._corrected_goal_x = 5 + self._COORDINATE_FRAME_OFFSET
        self._corrected_goal_y = 5 + self._COORDINATE_FRAME_OFFSET
        self._corrected_goal_z = 1

    def get_desired_command(self):
        with self._lock:
            rospy.logerr('TASK STARTS AT: ' + str(time.time()))

            if self._canceled:
                return (TaskCanceled(),)

            expected_time = rospy.Time.now() + self._PLANNING_LAG
            starting = self._linear_gen.expected_point_at_time(expected_time)

            _pose = starting.motion_point.pose.position
            self._vel_start = starting.motion_point.twist.linear

            self._corrected_start_x = _pose.x + self._COORDINATE_FRAME_OFFSET
            self._corrected_start_y = _pose.y + self._COORDINATE_FRAME_OFFSET
            self._corrected_start_z = _pose.z

            done = (abs(self._corrected_start_x-self._corrected_goal_x ) < .05 and
                    abs(self._corrected_start_y-self._corrected_goal_y) < .05 and
                    abs(self._corrected_start_z-self._corrected_goal_z) < .05)

            if done:
                if self._plan is not None:
                    return (TaskDone(), GlobalPlanCommand(self._plan))
                else:
                    return (TaskDone(),)

            if self._feedback_receieved or self._starting:
                self._starting = False
                self._feedback_receieved = False
                self.topic_buffer.make_plan_request(self._generate_request(expected_time),
                                                    self._feedback_callback)

                if self._feedback is not None and self._feedback.success:
                    # send LLM the plan we recieved
                    return (TaskRunning(), GlobalPlanCommand(self._plan))
                else:
                    # planning failed, nop
                    return (TaskRunning(), NopCommand())

            return (TaskRunning(), NopCommand())

    def _generate_request(self, expected_time):
        request = PlanGoal()
        request.header.stamp = expected_time

        start = MotionPointStamped()
        start.motion_point.pose.position.x = self._corrected_start_x
        start.motion_point.pose.position.y = self._corrected_start_y
        start.motion_point.pose.position.z = self._corrected_start_z

        start.motion_point.twist.linear.x = self._vel_start.x
        start.motion_point.twist.linear.y = self._vel_start.y
        start.motion_point.twist.linear.z = self._vel_start.z

        goal = MotionPointStamped()
        goal.motion_point.pose.position.x = self._corrected_goal_x
        goal.motion_point.pose.position.y = self._corrected_goal_y
        goal.motion_point.pose.position.z = self._corrected_goal_z

        request.start = start
        request.goal = goal
        return request

    def _feedback_callback(self, status, msg):
        with self._lock:
            rospy.logerr('FEEDBACK AT: ' + str(time.time()))
            self._feedback = msg
            self._plan = msg.plan
            self._feedback_receieved = True

    def cancel(self):
        rospy.loginfo('TestPlannerTask canceled')
        self._canceled = True
        return True

    def set_incoming_transition(self, transition):
        self._transition = transition

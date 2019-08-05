import math
import rospy
import tf2_ros
import threading

from .abstract_task import AbstractTask
from iarc_tasks.task_states import (TaskRunning,
                                    TaskDone,
                                    TaskCanceled,
                                    TaskAborted,
                                    TaskFailed)
from iarc_tasks.task_commands import NopCommand, GlobalPlanCommand

from iarc7_msgs.msg import PlanGoal, PlanAction, MotionPointStamped

class GoToRoombaState(object):
    INIT = 1
    WAITING = 2
    PLAN_RECEIVED = 3
    COMPLETING = 4

class GoToRoombaTask(AbstractTask):
    def __init__(self, task_request):
        super(GoToRoombaTask, self).__init__()

        # id of roomba to go to
        self._roomba_id = task_request.frame_id

        if self._roomba_id == '':
            raise ValueError('An invalid roomba id was provided to GoToRoombaTask')

        self._roomba_odometry = None

        self._plan = None
        self._feedback = None

        self._complete_time = None
        self._sent_plan_time = None

        self._x_position = None
        self._y_position = None

        self._starting_motion_point = None

        self._vel_start = None

        self._canceled = False;
        self._transition = None

        self._linear_gen = self.topic_buffer.get_linear_motion_profile_generator()

        self._lock = threading.RLock()

        try:
            self._MIN_MANEUVER_HEIGHT = rospy.get_param('~min_maneuver_height')
            self._PLANNING_TIMEOUT = rospy.Duration(rospy.get_param('~planning_timeout'))
            self._PLANNING_LAG = rospy.Duration(rospy.get_param('~planning_lag'))
            self._HEIGHT = rospy.get_param('~go_to_roomba_height')
            self._DONE_REPLAN_DIST = rospy.get_param('~done_replanning_radius')
        except KeyError as e:
            rospy.logerr('Could not lookup a parameter for go_to_roomba task')
            raise

        # make sure height param is not dumb
        if self._HEIGHT < self._MIN_MANEUVER_HEIGHT:
            raise ValueError('Requested z height was below the minimum maneuver height')

        self._state = GoToRoombaState.INIT

    def get_desired_command(self):
        with self._lock:
            if self._canceled:
                return (TaskCanceled(),)

            # cannot do anything until we have a roomba msg
            if not self.topic_buffer.has_roomba_message():
                return (TaskRunning(),NopCommand())

            # waiting on LLM to finish executing the plan
            if self._state == GoToRoombaState.COMPLETING:
                if (rospy.Time.now() + rospy.Duration(.10)) >= self._complete_time:
                    return (TaskDone(),)
                else:
                    return (TaskRunning(), NopCommand())

            if not self._check_roomba_in_sight():
                return (TaskFailed(msg='Cannnot see Roomba in Go To Roomba.'))

            expected_time = rospy.Time.now() + self._PLANNING_LAG

            self._starting_motion_point = self._linear_gen.expected_point_at_time(expected_time).motion_point

            _starting_pose = self._starting_motion_point.pose.position

            roomba_pose = self._roomba_odometry.pose.pose.position
            roomba_twist = self._roomba_odometry.twist.twist.linear

            x_traveled = roomba_twist.x * self._PLANNING_LAG.to_sec()
            y_traveled = roomba_twist.y * self._PLANNING_LAG.to_sec()

            self._goal_x = roomba_pose.x + x_traveled
            self._goal_y = roomba_pose.y + y_traveled

            _distance_to_goal = math.sqrt(
                        (_starting_pose.x-self._goal_x)**2 +
                        (_starting_pose.y-self._goal_y)**2)

            if _distance_to_goal < self._DONE_REPLAN_DIST:
                if self._plan is not None:
                    try:
                        self._complete_time = self._plan.motion_points[-1].header.stamp
                    except:
                        rospy.logerr('Planner returned an empty plan while GoToRoomba was in COMPLETING state')
                        return (TaskAborted(),)
                    self._state = GoToRoombaState.COMPLETING
                    return (TaskRunning(), GlobalPlanCommand(self._plan))
                else:
                    rospy.logerr('GoToRoombaTask: Plan is None but we are done')
                    return (TaskFailed(msg='Started too close to roomba to do anything'),)

            if self._state == GoToRoombaState.INIT:
                self._sent_plan_time = rospy.Time.now()
                self.topic_buffer.make_plan_request(self._generate_request(expected_time),
                                                    self._feedback_callback)
                self._state = GoToRoombaState.WAITING

            if self._state == GoToRoombaState.PLAN_RECEIVED:
                if self._feedback is not None:
                    self.topic_buffer.make_plan_request(
                        self._generate_request(expected_time, self._feedback.success),
                        self._feedback_callback)

                    self._state = GoToRoombaState.WAITING
                    if self._feedback.success:
                        # send LLM the plan we received
                        return (TaskRunning(), GlobalPlanCommand(self._plan))
                else:
                    rospy.logerr('GoToRoombaTask: In PLAN_RECEIVED state but no feedback')
                    return (TaskFailed(msg='In PLAN_RECEIVED state but no feedback'),)

            if (self._sent_plan_time is not None and
                (rospy.Time.now() - self._sent_plan_time) > self._PLANNING_TIMEOUT):
                return (TaskFailed(msg='GoToRoomba: planner took too long to plan'),)

            return (TaskRunning(), NopCommand())

    def _generate_request(self, expected_time, reset_timer=True):
        request = PlanGoal()
        request.header.stamp = expected_time

        start = MotionPointStamped()
        start.motion_point = self._starting_motion_point

        goal = MotionPointStamped()
        goal.motion_point.pose.position.x = self._goal_x
        goal.motion_point.pose.position.y = self._goal_y
        goal.motion_point.pose.position.z = self._HEIGHT

        request.start = start
        request.goal = goal

        if reset_timer:
            self._sent_plan_time = rospy.Time.now()

        return request

    def _feedback_callback(self, status, msg):
        with self._lock:
            self._feedback = msg
            self._plan = msg.plan
            self._state = GoToRoombaState.PLAN_RECEIVED

    def _check_roomba_in_sight(self):
        for odometry in self.topic_buffer.get_roomba_message().data:
            if odometry.child_frame_id == self._roomba_id:
                self._roomba_odometry = odometry
                return True
        return False

    def cancel(self):
        rospy.loginfo('GoToRoombaTask canceled')
        self._canceled = True
        return True

    def set_incoming_transition(self, transition):
        self._transition = transition

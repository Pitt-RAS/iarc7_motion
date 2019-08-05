import rospy
import tf2_ros
import actionlib
import time

from nav_msgs.msg import Odometry

from iarc7_msgs.msg import BoolStamped, OdometryArray, PlanAction

from iarc7_motion.linear_motion_profile_generator import LinearMotionProfileGenerator
from iarc_tasks.task_utilities.obstacle_avoid_helper import ObstacleAvoider
from iarc7_safety.iarc_safety_exception import IARCFatalSafetyException

class TaskTopicBuffer(object):
    def __init__(self):
        try:
            # startup timeout
            self._startup_timeout = rospy.Duration(rospy.get_param('~startup_timeout'))
        except KeyError as e:
            rospy.logerr('Could not lookup a parameter for task topic buffer')
            raise

        self._landed_message = None
        self._roomba_array = None
        self._drone_odometry = None
        self._obstacle_avoider = None

        self._landing_message_sub = rospy.Subscriber(
            'landing_detected', BoolStamped,
            self._receive_landing_status)

        self._roomba_status_sub = rospy.Subscriber(
            'roombas', OdometryArray,
            self._receive_roomba_status)

        self._current_velocity_sub = rospy.Subscriber(
            '/odometry/filtered', Odometry,
            self._current_velocity_callback)

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)
        self._motion_profile_generator = LinearMotionProfileGenerator.get_linear_motion_profile_generator()
        self._obstacle_avoider = ObstacleAvoider(self._tf_buffer)
        self._obstacle_avoider.wait_until_ready(self._startup_timeout)

        # client to planner Action Server
        self._planner_client = actionlib.SimpleActionClient('planner_request', PlanAction)

        if not self._planner_client.wait_for_server(self._startup_timeout):
            raise IARCFatalSafetyException(
                'TaskTopicBuffer could not initialize planner action client')

        self._task_message_dictionary = {}

        self._roomba_tracking_pub = rospy.Publisher(
            'roomba_tracking_status', Odometry,
            queue_size=10)

    def wait_until_ready(self, timeout):
        start_time = rospy.Time.now()
        rate = rospy.Rate(30)
        while not rospy.is_shutdown() and rospy.Time.now() - start_time < timeout:
            if (self.has_landing_message()
                    and self.has_odometry_message()
                    and self.has_roomba_message()):
                return
            rate.sleep()
        rospy.logerr('Have landing: %s', self.has_landing_message())
        rospy.logerr('Have odom: %s', self.has_odometry_message())
        rospy.logerr('Have roomba: %s', self.has_roomba_message())
        raise IARCFatalSafetyException('TaskTopicBuffer not ready')

    def make_plan_request(self, request, feedback_callback):
        self._planner_client.send_goal(request, done_cb=feedback_callback)

    def cancel_plan_goal(self):
        self._planner_client.cancel_goal()

    def _receive_roomba_status(self, data):
        self._roomba_array = data

    def _receive_landing_status(self, data):
        self._landed_message = data

    def _current_velocity_callback(self, data):
        self._drone_odometry = data

    def has_landing_message(self):
        return self._landed_message is not None

    def has_roomba_message(self):
        return self._roomba_array is not None

    def has_odometry_message(self):
        return self._drone_odometry is not None

    def get_landing_message(self):
        return self._landed_message

    def get_roomba_message(self):
        return self._roomba_array

    def get_roomba_odometry(self, id):
        if self.has_roomba_message():
            for odometry in self._roomba_array.data:
                if odometry.child_frame_id == id:
                    return True, odometry
        return False, None

    def get_odometry_message(self):
        return self._drone_odometry

    def get_tf_buffer(self):
        return self._tf_buffer

    def get_linear_motion_profile_generator(self):
        return self._motion_profile_generator

    def publish_roomba_tracking_status(self, pose):
        self._roomba_tracking_pub.publish(pose)

    def get_task_message_dictionary(self):
        return self._task_message_dictionary

    def get_obstacle_avoider(self):
        return self._obstacle_avoider

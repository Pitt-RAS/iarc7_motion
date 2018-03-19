
import threading
import weakref
import time
import rospy
import actionlib

from actionlib_msgs.msg import GoalStatus
from actionlib.action_client import AcionClient, CommState, get_name_of_constant

from iarc_tasks.takeoff_task import TakeoffTask
from iarc_tasks.land_task import LandTask
from iarc_tasks.test_task import TestTask
from iarc_tasks.xyztranslation_task import XYZTranslationTask
from iarc_tasks.track_roomba_task import TrackRoombaTask
from iarc_tasks.hit_roomba_task import HitRoombaTask
from iarc_tasks.block_roomba_task import BlockRoombaTask
from iarc_tasks.hold_position_task import HoldPositionTask
from iarc_tasks.height_recovery_task import HeightRecoveryTask
from iarc_tasks.velocity_task import VelocityTask

class SimpleGoalState:
	PENDING = 0
	ACTIVE = 1
	DONE = 2

SimpleGoalState.to_string = classmethod(get_name_of_constant)

# (DELETET THIS) Note: cb means callback

class IarcTaskActionClient:

	def __init__(self, comm_state_machine):
		self._action_name = 'motion_planner_client'

		self._action_client = actionlib.SimpleActionClient(self._action_client,
													QuadMoveAction)
		self._simple_state = SimpleGoalState.DONE
		self._call_back_catcher = None
		self._done_condition = threading.Condition()
		self._goal_queue = []

	def wait_for_server(self, timeout=rospy.Duration()):

		return self._action_client.wait_for_server(timeout)


	def send_goal(self, goal, done_cb=None, active_cb=None, feedback_cb=None):
		self._goal_queue.append(goal)
		if 

	def wait_for_result(self, timeout=rospy.Duration()):
		

		#finish this

	def get_result(self):
		if not self._call_back_catcher:
			rospy.logerr("Called get_result when no goal was in queue")
import rospy
import tf2_ros
import tf2_geometry_msgs
import threading

from .abstract_task import AbstractTask
from iarc_tasks.task_states import (TaskRunning,
                                    TaskDone,
                                    TaskCanceled,
                                    TaskAborted,
                                    TaskFailed)
from iarc_tasks.task_commands import (VelocityCommand, NopCommand)

from task_utilities.translate_stop_planner import TranslateStopPlanner

class GoToRoombaState(object):
    init = 0
    translate = 1
    waiting = 2

class GoToRoombaTask(AbstractTask):

    def __init__(self, task_request):
        super(GoToRoombaTask, self).__init__()

        # id of roomba to go to
        self._roomba_id = task_request.frame_id

        if self._roomba_id == '':
            raise ValueError('An invalid roomba id was provided to GoToRoombaTask')

        self._roomba_odometry = None

        self._canceled = False;
        self._transition = None

        self._lock = threading.RLock()

        self._x_position = None
        self._y_position = None

        try:
            self._TRANSFORM_TIMEOUT = rospy.get_param('~transform_timeout')
            self._MIN_MANEUVER_HEIGHT = rospy.get_param('~min_maneuver_height')
            self._z_position = rospy.get_param('~track_roomba_height')
            ending_radius = rospy.get_param('~go_to_roomba_tolerance')
        except KeyError as e:
            rospy.logerr('Could not lookup a parameter for go_to_roomba task')
            raise

        # Check that we aren't being requested to go below the minimum maneuver height
        # Error straight out if that's the case. If we are currently below the minimum height
        # It will be caught and handled on the next update
        if self._z_position < self._MIN_MANEUVER_HEIGHT :
            raise ValueError('Requested z height was below the minimum maneuver height')

        self._path_holder = TranslateStopPlanner(self._x_position,
                                                    self._y_position,
                                                    self._z_position,
                                                    ending_radius)

        self._state = GoToRoombaState.init

    def get_desired_command(self):
        with self._lock:
            if self._canceled:
                return (TaskCanceled(),)
            if self._state == GoToRoombaState.init:
                if (not self.topic_buffer.has_roomba_message()
                  or not self.topic_buffer.has_odometry_message()):
                    self._state = GoToRoombaState.waiting
                else:
                    self._state = GoToRoombaState.translate

            if self._state == GoToRoombaState.waiting:
                if (not self.topic_buffer.has_roomba_message()
                  or not self.topic_buffer.has_odometry_message()):
                    return (TaskRunning(),NopCommand())
                else:
                    self._state = GoToRoombaState.translate

            if self._state == GoToRoombaState.translate:
                try:
                    transStamped = self.topic_buffer.get_tf_buffer().lookup_transform(
                            'map',
                            'quad',
                            rospy.Time(0),
                            rospy.Duration(self._TRANSFORM_TIMEOUT))
                except (tf2_ros.LookupException,
                        tf2_ros.ConnectivityException,
                        tf2_ros.ExtrapolationException) as ex:
                    rospy.logerr('GoToRoombaTask: Exception when looking up transform')
                    rospy.logerr(ex.message)
                    return (TaskAborted(msg = 'Exception when looking up transform during go_to_roomba'),)

                if (transStamped.transform.translation.z > self._MIN_MANEUVER_HEIGHT):
                    if not self._check_roomba_in_sight():
                        return (TaskAborted(msg='The provided roomba is not in sight of quad'),)

                    roomba_x = self._roomba_odometry.pose.pose.position.x
                    roomba_y = self._roomba_odometry.pose.pose.position.y

                    self._path_holder.reinit_translation_stop_planner(roomba_x,
                                                                      roomba_y,
                                                                      self._z_position)

                    hold_twist = self._path_holder.get_xyz_hold_response()

                    if not self._path_holder.is_done():
                        return (TaskRunning(), VelocityCommand(hold_twist))
                    else:
                        return (TaskDone(), VelocityCommand(hold_twist))
                else:
                    return (TaskFailed(msg='Fell below minimum manuever height during translation'),)

            return (TaskAborted(msg='Impossible state in go to roomba task reached'))

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

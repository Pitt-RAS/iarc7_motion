import math
import rospy
import tf2_ros
import tf2_geometry_msgs
import threading

from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped

from .abstract_task import AbstractTask
from iarc_tasks.task_states import (TaskRunning,
                                    TaskDone,
                                    TaskCanceled,
                                    TaskAborted,
                                    TaskFailed)
from iarc_tasks.task_commands import (VelocityCommand, NopCommand)

from translate_stop_planner import TranslateStopPlanner

class GoToRoombaState(object):
    init = 0
    translate = 1

class GoToRoombaTask(AbstractTask):

    def __init__(self, task_request):
        super(GoToRoombaTask, self).__init__()

        self._roomba_id = task_request.frame_id + '/base_link'

        if self._roomba_id is None:
            raise ValueError('A null roomba id was provided')

        self._roomba_odometry = None
        self._roomba_point = None

        self._canceled = False;
        self._last_update_time = None
        self._current_velocity = None
        self._transition = None

        self._lock = threading.RLock()

        self._x_position = task_request.x_position
        self._y_position = task_request.y_position


        try:
            self._TRANSLATION_XYZ_TOLERANCE = rospy.get_param('~translation_xyz_tolerance')
            self._TRANSFORM_TIMEOUT = rospy.get_param('~transform_timeout')
            self._MIN_MANEUVER_HEIGHT = rospy.get_param('~min_maneuver_height')
            self._MAX_HORIZ_SPEED = rospy.get_param('~max_translation_speed')
            self._MAX_TASK_DIST = rospy.get_param('~max_roomba_dist')
            self._K_X = rospy.get_param('~k_term_tracking_x')
            self._K_Y = rospy.get_param('~k_term_tracking_y')
            self._z_position = rospy.get_param('~track_roomba_height')
        except KeyError as e:
            rospy.logerr('Could not lookup a parameter for xyztranslation task')
            raise

        # Check that we aren't being requested to go below the minimum maneuver height
        # Error straight out if that's the case. If we are currently below the minimum height
        # It will be caught and handled on the next update
        if self._z_position < self._MIN_MANEUVER_HEIGHT :
            raise ValueError('Requested z height was below the minimum maneuver height')

        # fc status?
        self._path_holder = TranslateStopPlanner(self._x_position,
                                                    self._y_position,
                                                    self._z_position)

        self._state = GoToRoombaState.init


    def get_desired_command(self):
        with self._lock:
            if self._canceled:
                return (TaskCanceled(),)
            if self._state == GoToRoombaState.init:
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

                if(transStamped.transform.translation.z > self._MIN_MANEUVER_HEIGHT):
                    
                    if not self._check_roomba_in_sight():
                        return (TaskAborted(msg='The provided roomba is not in sight of quad'),)
                    elif self._on_ground():
                        return (TaskDone(),)

                    try:
                        roomba_transform = self.topic_buffer.get_tf_buffer().lookup_transform(
                                        'map',
                                        self._roomba_id,
                                        rospy.Time(0),
                                        rospy.Duration(self._TRANSFORM_TIMEOUT))
                        roomba_check_pos = self.topic_buffer.get_tf_buffer().lookup_transform(
                                         'level_quad',
                                         self._roomba_id,
                                         rospy.Time(0),
                                         rospy.Duration(self._TRANSFORM_TIMEOUT))
                    except (tf2_ros.LookupException,
                            tf2_ros.ConnectivityException,
                            tf2_ros.ExtrapolationException) as ex:
                        rospy.logerr('ObjectTrackTask: Exception when looking up transform')
                        rospy.logerr(ex.message)
                        return (TaskAborted(msg='Exception when looking up transform during go to roomba'),)
                        


                    # Create point at drones center
                    stamped_point = PointStamped()
                    stamped_point.point.x = 0
                    stamped_point.point.y = 0
                    stamped_point.point.z = 0

                    # returns point distances of roomba to center point of the map
                    self._roomba_point_map = tf2_geometry_msgs.do_transform_point(
                                                            stamped_point, roomba_transform)

                    self._roomba_point_level_quad = tf2_geometry_msgs.do_transform_point(
                                                            stamped_point, roomba_check_pos)


                    roomba_x_velocity = self._roomba_odometry.twist.twist.linear.x
                    roomba_y_velocity = self._roomba_odometry.twist.twist.linear.y

                    x_vel_target = self._roomba_point_map.point.x + roomba_x_velocity
                    y_vel_target = self._roomba_point_map.point.y + roomba_y_velocity
                    
                    self._path_holder.reinitialize_translation_stop_planner(x_vel_target,
                                                                            y_vel_target,
                                                                            self._z_position)


                    """ quarantine end"""
                    hold_twist = self._path_holder.get_xyz_hold_response()
                    if self._check_max_roomba_range():
                        return (TaskDone(), VelocityCommand(hold_twist))
                    elif not self._path_holder.is_done():
                        return (TaskRunning(), VelocityCommand(hold_twist))
                    else:
                        return (TaskDone(), VelocityCommand(hold_twist))
                else:
                    return (TaskFailed(msg='Fell below minimum manuever height during translation'),)


            return (TaskAborted(msg='Impossible state in takeoff task reached'))

    """ qurantine methods """
    def _check_roomba_in_sight(self):
        for odometry in self.topic_buffer.get_roomba_message().data:
            if odometry.child_frame_id == self._roomba_id:
                self._roomba_odometry = odometry
                return True
        return False

    def _check_max_roomba_range(self,):
        _distance_to_roomba = math.sqrt(self._roomba_point_level_quad.point.x**2 +
                            self._roomba_point_level_quad.point.y**2)
        return (_distance_to_roomba <= self._MAX_TASK_DIST * 0.75)

    def _on_ground(self):
        return self.topic_buffer.get_landing_message().data
    """ quarantive methods end """

    def cancel(self):
        rospy.loginfo('GoToRoombaTask canceled')
        self._canceled = True
        return True

    def set_incoming_transition(self, transition):
        self._transition = transition

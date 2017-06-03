#!/usr/bin/env python

import math
import rospy
import tf2_ros
import tf2_geometry_msgs

from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped


from iarc7_msgs.msg import OdometryArray

from .abstract_task import AbstractTask
from iarc_tasks.task_states import (TaskRunning,
                                    TaskDone,
                                    TaskCanceled,
                                    TaskAborted,
                                    TaskFailed)
from iarc_tasks.task_commands import (VelocityCommand,
                                      ArmCommand,
                                      NopCommand)

from height_holder import HeightHolder

class TrackObjectTaskState:
    init = 0
    track = 1
    waiting = 2

class TrackRoombaTask(AbstractTask):

    def __init__(self, task_dictionary):

        self.roomba_id = task_dictionary['frame_id']

        self.roomba_id = self.roomba_id  + '/base_link'
        
        self.k_x = .5
        self.k_y = .5

        if self.roomba_id is None or len(self.roomba_id) < 2:
            raise ValueError('An invalid or null roomba id was provided')

        self.roomba_odometry = None
        self._roomba_array = None
        self.roomba_point = None

        self.roomba_found = False
        self._canceled = False

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)  

        self._roomba_status_sub = rospy.Subscriber('roombas', OdometryArray, self._receive_roomba_status)

        try:
            self._TRANSFORM_TIMEOUT = rospy.get_param('~transform_timeout')
            self._MIN_MANEUVER_HEIGHT = rospy.get_param('~min_maneuver_height')
            self._TRACK_HEIGHT = rospy.get_param('~track_roomba_height')
            self._MAX_TRANSLATION_SPEED = rospy.get_param('~max_translation_speed')
        except KeyError as e:
            rospy.logerr('Could not lookup a parameter for track roomba task')
            raise

        if self._TRACK_HEIGHT < self._MIN_MANEUVER_HEIGHT:
            raise ValueError('Track Roomba height was below the minimum maneuver height')

        self._z_holder = HeightHolder()

        self._state = TrackObjectTaskState.init

        while self._roomba_array is None:
            pass

    def _receive_roomba_status(self, data):
        self._roomba_array = data

    def get_desired_command(self):
        if self._state == TrackObjectTaskState.init or self._state == TrackObjectTaskState.waiting:
            self._state = TrackObjectTaskState.track

        if self._canceled:
            return (TaskCanceled(),)

        if self._roomba_array is None:
            self._state = TrackObjectTaskState.waiting
            return (TaskRunning(), NopCommand())

        if self._state == TrackObjectTaskState.track:
            try:
                roomba_transform = self._tf_buffer.lookup_transform(
                                    'level_quad',
                                    self.roomba_id,
                                    rospy.Time.now(),
                                    rospy.Duration(self._TRANSFORM_TIMEOUT))
            except (tf2_ros.LookupException,
                    tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException) as ex:
                rospy.logerr('ObjectTrackTask: Exception when looking up transform')
                rospy.logerr(ex.message)
                return (TaskAborted(msg='Exception when looking up transform during roomba track'),)

            stamped_point = PointStamped()
            stamped_point.point.x = 0
            stamped_point.point.y = 0
            stamped_point.point.z = 0

            self.roomba_point =tf2_geometry_msgs.do_transform_point(stamped_point, roomba_transform)



            if not self._check_roomba_range():
                return (TaskAborted(msg='The provided roomba is not found or not within a meter of the quad'),)

            x_error = self.roomba_point.point.x * self.k_x + self.roomba_odometry.twist.twist.linear.x

            y_error = self.roomba_point.point.y * self.k_y + self.roomba_odometry.twist.twist.linear.y

            z_error = self._z_holder.get_height_hold_response()

            if abs(x_error) > self._MAX_TRANSLATION_SPEED:
                x_error = x_error/abs(x_error) * self._MAX_TRANSLATION_SPEED

            if abs(y_error) > self._MAX_TRANSLATION_SPEED:
                y_error = y_error/abs(y_error) * self._MAX_TRANSLATION_SPEED

            velocity = TwistStamped()
            velocity.header.frame_id = 'level_quad'
            velocity.header.stamp = rospy.Time.now()
            velocity.twist.linear.x = x_error
            velocity.twist.linear.y = y_error
            velocity.twist.linear.z = z_error
            
            return (TaskRunning(), VelocityCommand(velocity)) 

    def _check_roomba_range(self):
        for odometry in self._roomba_array.data:
            if odometry.child_frame_id == self.roomba_id:
                self.roomba_odometry = odometry
                self.roomba_found =  True 
        return (abs(self.roomba_point.point.x) < 1.0 and abs(self.roomba_point.point.y) < 1.0 and self.roomba_found)

    def cancel(self):
        rospy.loginfo('TrackObject Task canceled')
        self._canceled = True

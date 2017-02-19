#!/usr/bin/env python

import math
import rospy
import tf2_ros

from geometry_msgs.msg import TwistStamped

from iarc7_msgs.msg import FlightControllerStatus

from .abstract_task import AbstractTask
from .task_state import TaskState

class LandTaskState:
    init = 0
    land = 1
    disarm = 2

class LandTask(AbstractTask):

    def __init__(self, actionvalues_dict):
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)
        self._canceled = False;

        try:
            self._LAND_VELOCITY = rospy.get_param('~land_velocity')
            self._LAND_HEIGHT_TOLERANCE = rospy.get_param('~land_height_tolerance')
        except KeyError as e:
            rospy.logerr('Could not lookup a parameter for land task')
            raise
        
        self._fc_status = None
        self._fc_status_sub = rospy.Subscriber('fc_status', FlightControllerStatus, self._receive_fc_status)
        self._state = LandTaskState.init
        self._disarm_request_success = False

    def _receive_fc_status(self, data):
        self._fc_status = data

    def disarm_callback(self, data):
        self._disarm_request_success = data.success

    def get_desired_command(self):
        if self._canceled:
            return (TaskState.canceled,)

        if self._state == LandTaskState.init:
            # Check if we have a fc status
            if self._fc_status is None:
                return (TaskState.running, 'velocity', TwistStamped())
            # Check that auto pilot is enabled
            if not self._fc_status.auto_pilot:
                return (TaskState.failed, 'flight controller not allowing auto pilot')
            # Check that the FC is not already armed
            if not self._fc_status.armed:
                return (TaskState.failed, 'flight controller is not armed, how are we in the air??')
            # All is good change state to land
            self._state = LandTaskState.land

        # Enter the takeoff phase
        if self._state == LandTaskState.land:
            try:
                transStamped = self._tf_buffer.lookup_transform('map', 'quad', rospy.Time(0))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as ex:
                rospy.logerr('LandTask: Exception when looking up transform')
                rospy.logerr(ex.message)
                return (TaskState.aborted, 'Exception when looking up transform during landing')

            # Check if we reached the target height
            if(transStamped.transform.translation.z < self._LAND_HEIGHT_TOLERANCE):
                self._state = LandTaskState.disarm
            else:
                velocity = TwistStamped()
                velocity.header.frame_id = 'level_quad'
                velocity.header.stamp = rospy.Time.now()
                velocity.twist.linear.z = self._LAND_VELOCITY
                return (TaskState.running, 'velocity', velocity)

        # Enter the disarming request stage
        if self._state == LandTaskState.disarm:
            # Check if disarm succeeded
            if self._disarm_request_success:
                return (TaskState.done, 'nop')
            else:
                return (TaskState.running, 'disarm', self.disarm_callback)

        # Impossible state reached
        return (TaskState.aborted, 'Impossible state in takeoff task reached')

    def cancel(self):
        rospy.loginfo('TakeoffTask canceled')
        self._canceled = True

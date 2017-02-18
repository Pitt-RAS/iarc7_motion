#!/usr/bin/env python
import rospy
from abstract_task import AbstractTask
from task_state import TaskState
from geometry_msgs.msg import TwistStamped

import tf2_ros

import math

from iarc7_msgs.msg import FlightControllerStatus

class TakeoffTaskState:
    init = 0
    request_arm = 1
    pause_before_takeoff = 2
    takeoff = 3

class TakeoffTask(AbstractTask):

    def __init__(self, actionvalues_dict):
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)
        self._canceled = False;

        self._takeoff_height = actionvalues_dict['takeoff_height']
        try:
            self._TAKEOFF_VELOCITY = rospy.get_param('~takeoff_velocity')
            self._TAKEOFF_HEIGHT_TOLERANCE = rospy.get_param('~takeoff_height_tolerance')
            self._DELAY_BEFORE_TAKEOFF = rospy.get_param('~delay_before_takeoff')
            self._MAX_TAKEOFF_START_HEIGHT = rospy.get_param('~max_takeoff_start_height')
        except KeyError as e:
            rospy.logerr('Could not lookup a parameter for takeoff task')
            raise
        
        self._fc_status = None
        self._fc_status_sub = rospy.Subscriber('fc_status', FlightControllerStatus, self._receive_fc_status)
        self._state = TakeoffTaskState.init
        self._arm_request_success = False

    def _receive_fc_status(self, data):
        self._fc_status = data

    def arm_callback(self, data):
        self._arm_request_success = data.success

    def get_desired_command(self):
        if self._canceled:
            return (TaskState.canceled,)

        if self._state == TakeoffTaskState.init:
            # Check if we have an fc status
            if self._fc_status is None:
                return (TaskState.running, 'velocity', TwistStamped())
            # Check that auto pilot is enabled
            if not self._fc_status.auto_pilot:
                return (TaskState.failed, 'flight controller not allowing auto pilot')
            # Check that the FC is not already armed
            if self._fc_status.armed:
                return (TaskState.failed, 'flight controller armed prior to takeoff')

            try:
                trans = self._tf_buffer.lookup_transform('map', 'quad', rospy.Time(0))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as ex:
                rospy.logerr('Takeofftask: Exception when looking up transform')
                rospy.logerr(ex.message)
                # Make sure not to leave the init state
                self._state = TakeoffTaskState.init
                return (TaskState.aborted, 'Exception when looking up transform')

            if trans.transform.translation.z > self._MAX_TAKEOFF_START_HEIGHT:
                return (TaskState.failed, 'quad is too high to takeoff')

            # All is good change state to request arm
            self._state = TakeoffTaskState.request_arm

        # Enter the arming request stage
        if self._state == TakeoffTaskState.request_arm:
            # Check that the FC is not already armed
            if self._arm_request_success:
                self.pause_start_time = rospy.Time.now()
                self._state = TakeoffTaskState.pause_before_takeoff
            else:
                return (TaskState.running, 'arm', self.arm_callback)

        # Pause before ramping up the motors
        if self._state == TakeoffTaskState.pause_before_takeoff:
            if rospy.Time.now() - self.pause_start_time > rospy.Duration(self._DELAY_BEFORE_TAKEOFF):
                self._state = TakeoffTaskState.takeoff
            else:
                return (TaskState.running, 'velocity', TwistStamped())

        # Enter the takeoff phase
        if self._state == TakeoffTaskState.takeoff:
            try:
                transStamped = self._tf_buffer.lookup_transform('map', 'quad', rospy.Time(0))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as ex:
                rospy.logerr('Takeofftask: Exception when looking up transform')
                rospy.logerr(ex.message)
                return (TaskState.aborted, 'Exception when looking up transform during takeoff')
        
            # Check if we reached the target height
            if(transStamped.transform.translation.z > self._takeoff_height - self._TAKEOFF_HEIGHT_TOLERANCE):
                return (TaskState.done, 'velocity', TwistStamped())

            velocity = TwistStamped()
            velocity.header.frame_id = 'level_quad'
            velocity.header.stamp = rospy.Time.now()
            velocity.twist.linear.z = self._TAKEOFF_VELOCITY
            return (TaskState.running, 'velocity', velocity)

        # Impossible state reached
        return (TaskState.aborted, 'Impossible state in takeoff task reached')


    def cancel(self):
        rospy.loginfo('TakeoffTask canceled')
        self._canceled = True

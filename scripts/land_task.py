#!/usr/bin/env python
import rospy
from abstract_task import AbstractTask
from task_state import TaskState
from geometry_msgs.msg import TwistStamped

import rospy
import tf2_ros

import math

from iarc7_msgs.msg import FlightControllerStatus
from geometry_msgs.msg import TwistStamped

class LandTaskState:
    init = 0
    land = 1
    disarm = 2

class LandTask(AbstractTask):

    def __init__(self, actionvalues_dict):
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.canceled = False;

        try:
            self.LAND_VELOCITY = rospy.get_param('~land_velocity')
            self.LAND_HEIGHT_TOLERANCE = rospy.get_param('~land_height_tolerance')
        except KeyError as e:
            rospy.logerr('Could not lookup a parameter for takeoff task')
            raise
        
        self.fc_status = None
        self.fc_status_sub = rospy.Subscriber('fc_status', FlightControllerStatus, self._receive_fc_status)
        self.state = LandTaskState.init
        self.disarm_request_success = False

    def _receive_fc_status(self, data):
        self.fc_status = data

    def disarm_callback(self, data):
        self.disarm_request_success = data.success

    def get_desired_command(self):
        if self.canceled:
            return (TaskState.canceled)

        if self.state == LandTaskState.init:
            if self.fc_status != None:
                # Check that auto pilot is enabled
                if not self.fc_status.auto_pilot:
                    return (TaskState.failed, 'flight controller not allowing auto pilot')
                # Check that the FC is not already armed
                elif not self.fc_status.armed:
                    return (TaskState.failed, 'flight controller is not armed, how are we in the air??')
                # All is good change state to land
                else:
                    self.state = LandTaskState.land
            else:
                return (TaskState.running, 'velocity', TwistStamped())

        # Enter the takeoff phase
        if self.state == LandTaskState.land:
            try:
                transStamped = self.tf_buffer.lookup_transform('map', 'quad', rospy.Time(0))
                # Check if we reached the target height
                if(transStamped.transform.translation.z < self.LAND_HEIGHT_TOLERANCE):
                    self.state = LandTaskState.disarm

                velocity = TwistStamped()
                velocity.header.frame_id = 'level_quad'
                velocity.header.stamp = rospy.Time.now()
                velocity.twist.linear.z = self.LAND_VELOCITY
                return (TaskState.running, 'velocity', velocity)

            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as ex:
                rospy.logerr('Takeofftask: Exception when looking up transform')
                rospy.logerr(ex.message)
                return (TaskState.aborted, 'Exception when looking up transform during takeoff')

        # Enter the disarming request stage
        if self.state == LandTaskState.disarm:
            # Check if disarm succeeded
            if self.disarm_request_success:
                return (TaskState.done, 'velocity', TwistStamped())
            else:
                return (TaskState.running, 'disarm', self.disarm_callback)

        # Impossible state reached
        return (TaskState.aborted, 'Impossible state in takeoff task reached')

    def cancel(self):
        rospy.loginfo('TakeoffTask canceled')
        self.canceled = True

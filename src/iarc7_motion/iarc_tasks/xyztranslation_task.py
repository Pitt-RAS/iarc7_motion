import math
import rospy
import tf2_ros

from geometry_msgs.msg import TwistStamped

from .abstract_task import AbstractTask
from iarc_tasks.task_states import (TaskRunning,
                                    TaskDone,
                                    TaskCanceled,
                                    TaskAborted,
                                    TaskFailed)
from iarc_tasks.task_commands import (VelocityCommand, NopCommand)

from position_holder import PositionHolder

class XYZTranslationTaskState:
    init = 0
    translate = 1

class XYZTranslationTask(AbstractTask):

    def __init__(self, actionvalues_dict):
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)
        self._canceled = False;

        self._x_position = actionvalues_dict['x_position']
        self._y_position = actionvalues_dict['y_position']
        self._z_poisiton = actionvalues_dict['z_position']

        try:
            self._TRANSLATION_XYZ_TOLERANCE = rospy.get_param('~translation_xyz_tolerance');
            self._TRANSFORM_TIMEOUT = rospy.get_param('~transform_timeout')
            self._MIN_MANEUVER_HEIGHT = rospy.get_param('~min_maneuver_height')
        except KeyError as e:
            rospy.logerr('Could not lookup a parameter for xyztranslation task')
            raise

        # fc status?
        self._path_holder = PositionHolder()
        self._state = XYZTranslationTaskState.init

    def get_desired_command(self):
        if self._canceled:
            return (TaskCanceled(),) # ask about syntax

        if self._state == XYZTranslationTaskState.init:
            self._state = LandTaskState.translate

        if self._state == XYZTranslationTaskState.translate:
            try:
                transStamped = self._tf_buffer.lookup_transform('map', 'quad', Rospy.Time.now(), self._TRANSFORM_TIMEOUT)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as ex:
                rospy.logerr('LandTask: Exception when looking up transform')
                rospy.logerr(ex.message)
                return (TaskAborted(msg = 'Exception when looking up transform during xyztranslation'))

            if (transStamped.transform.translation.x < self._TRANSLATION_XYZ_TOLERANCE
                and transStamped.transform.translation.y < self._TRANSLATION_XYZ_TOLERANCE
                and transStamped.transform.translation.z < self._TRANSLATION_XYZ_TOLERANCE):
                    return (TaskDone(), NopCommand())
            else:
                if(transStamped.transform.translation.z > self._MIN_MANEUVER_HEIGHT):
                    hold_twist = self._path_holder.get_xyz_hold_response()
                    return (TaskRunning(), VelocityCommand(hold_twist))
                else:
                    velocity = TwistStamped()
                    velocity.header.frame_id = 'level_quad'
                    velocity.header.stamp = rospy.Time.now()
                    # velocity.twist.linear.z = self._LAND_VELOCITY # TODO
                    return (TaskRunning(), VelocityCommand(velocity))

        return (TaskAborted(msg='Impossible state in takeoff task reached'))

    def cancel(self):
        rospy.loginfo('XYZTranslationTask canceled')
        self._canceled = True

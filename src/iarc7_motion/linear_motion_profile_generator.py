#!/usr/bin/env python

import rospy
import numpy as np

from iarc7_msgs.msg import MotionPointStamped, MotionPointStampedArray

def np_to_msg(array, msg):
    msg.x = array[0]
    msg.y = array[1]
    msg.z = array[2]
    return msg

def msg_to_np(msg):
    return np.array([msg.x, msg.y, msg.z])

def interpolate(first, second, fraction):
    return fraction * (second - first) + first

def interpolate_motion_points(first, second, time):
    fraction = ((time - first.header.stamp)
                / (second.header.stamp - first.header.stamp))

    result = MotionPointStamped()
    result.header.stamp = time

    result.motion_point.pose.position.x = interpolate(
                      first.motion_point.pose.position.x,
                      second.motion_point.pose.position.x,
                      fraction)
    result.motion_point.pose.position.y = interpolate(
                      first.motion_point.pose.position.y,
                      second.motion_point.pose.position.y,
                      fraction)
    result.motion_point.pose.position.z = interpolate(
                      first.motion_point.pose.position.z,
                      second.motion_point.pose.position.z,
                      fraction)

    result.motion_point.twist.linear.x = interpolate(
                      first.motion_point.twist.linear.x,
                      second.motion_point.twist.linear.x,
                      fraction)
    result.motion_point.twist.linear.y = interpolate(
                      first.motion_point.twist.linear.y,
                      second.motion_point.twist.linear.y,
                      fraction)
    result.motion_point.twist.linear.z = interpolate(
                      first.motion_point.twist.linear.z,
                      second.motion_point.twist.linear.z,
                      fraction)

    result.motion_point.accel.linear.x = interpolate(
                      first.motion_point.accel.linear.x,
                      second.motion_point.accel.linear.x,
                      fraction)
    result.motion_point.accel.linear.y = interpolate(
                      first.motion_point.accel.linear.y,
                      second.motion_point.accel.linear.y,
                      fraction)
    result.motion_point.accel.linear.z = interpolate(
                      first.motion_point.accel.linear.z,
                      second.motion_point.accel.linear.z,
                      fraction)

    return result

class LinearMotionProfileGenerator(object):
    def __init__(self, start_motion_point):
        self._last_motion_plan = None
        self._start_motion_point = start_motion_point

        try:
            self._TARGET_ACCEL = rospy.get_param('~linear_motion_profile_acceleration')
            self._PLAN_DURATION = rospy.get_param('~linear_motion_profile_duration')
            self._PROFILE_TIMESTEP = rospy.get_param('~linear_motion_profile_timestep')
        except KeyError as e:
            rospy.logerr('Could not lookup a parameter for linear motion profile generator')
            raise

        self._last_stamp = rospy.Time.now()

    def _get_start_point(self, time):
        # Get the starting point from the current motion plane
        if self._start_motion_point is not None:
            start_motion_point = self._start_motion_point
            self._start_motion_point = None

            # Check that the time is at least equal to the current time
            # before generating a plan
            if start_motion_point.header.stamp < rospy.Time.now():
                start_motion_point.header.stamp = rospy.Time.now()
            return start_motion_point

        else:
            # Make sure a starting point newer than the last sent time is sent
            for i in range(1, len(self._last_motion_plan.motion_points)):
                if self._last_motion_plan.motion_points[i].header.stamp > time:
                    first_point = self._last_motion_plan.motion_points[i-1]
                    second_point = self._last_motion_plan.motion_points[i]
                    return interpolate_motion_points(first_point, second_point, time)

            # A point was not sent before the buffer ran out
            # Use the oldest and reset the timestamp
            self._last_motion_plan.motion_points[-1].header.stamp = rospy.Time.now()
            return self._last_motion_plan.motion_points[-1]

    def get_velocity_plan(self, velocity_command):

        start_point = self._get_start_point(velocity_command.target_twist.header.stamp)

        rospy.logwarn(velocity_command.target_twist.header.stamp - self._last_stamp)
        self._last_stamp = velocity_command.target_twist.header.stamp

        p_start = msg_to_np(start_point.motion_point.pose.position)
        v_start = msg_to_np(start_point.motion_point.twist.linear)
        v_desired = msg_to_np(velocity_command.target_twist.twist.linear)
        v_delta = v_desired - v_start

        a_target = self._TARGET_ACCEL * v_delta / np.linalg.norm(v_delta)

        acceleration_time = min(np.linalg.norm(v_delta) / self._TARGET_ACCEL, self._PLAN_DURATION)
        steady_velocity_time = self._PLAN_DURATION - acceleration_time

        accel_steps = np.floor(acceleration_time / self._PROFILE_TIMESTEP)
        vel_steps = np.floor(steady_velocity_time / self._PROFILE_TIMESTEP)

        velocities = [v_start]

        if accel_steps > 0:
            accel_times = (np.linspace(0.0, accel_steps*self._PROFILE_TIMESTEP, accel_steps, endpoint=True)
                     * accel_steps
                     * self._PROFILE_TIMESTEP)
            velocities.extend([a_target * accel_time for accel_time in accel_times])

        velocities.extend([v_desired for i in range(0, int(vel_steps))])

        velocities = np.array(velocities)

        accelerations = np.diff(velocities, axis=0) / self._PROFILE_TIMESTEP
        positions = np.cumsum(velocities, axis=0) * self._PROFILE_TIMESTEP

        rospy.loginfo(positions)

        plan = MotionPointStampedArray()

        motion_point = MotionPointStamped()
        motion_point.header.stamp = start_point.header.stamp
        np_to_msg(accelerations[0], motion_point.motion_point.accel.linear)
        np_to_msg(v_start, motion_point.motion_point.twist.linear)
        np_to_msg(p_start, motion_point.motion_point.pose.position)
        plan.motion_points.append(motion_point)

        # Don't output the last point since we don't have an
        # acceleration for it
        for i in range(1, velocities.shape[0]-1):
            motion_point = MotionPointStamped()
            motion_point.header.stamp = (start_point.header.stamp
                                         + rospy.Duration(i * self._PROFILE_TIMESTEP))
            np_to_msg(accelerations[i], motion_point.motion_point.accel.linear)
            np_to_msg(velocities[i] + v_start, motion_point.motion_point.twist.linear)
            np_to_msg(positions[i-1] + p_start, motion_point.motion_point.pose.position)
            plan.motion_points.append(motion_point)

        self._last_motion_plan = plan
        return plan

    def reinitialize_start_point(self, start_motion_point):
        self._start_motion_point = start_motion_point

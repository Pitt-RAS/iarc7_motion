#!/usr/bin/env python

import rospy
import numpy as np

from geometry_msgs.msg import PoseStamped

from nav_msgs.msg import Path

from iarc7_msgs.msg import MotionPointStamped, MotionPointStampedArray
import cPickle as pickle

# Convert a 3 element numpy array to a Vector3 message
def np_to_msg(array, msg):
    msg.x = array[0]
    msg.y = array[1]
    msg.z = array[2]
    return msg


# Convert a Vector3 message to 3 element numpy array
def msg_to_np(msg):
    return np.array([msg.x, msg.y, msg.z])


# Interpolate two values
def interpolate(first, second, fraction):
    return fraction * (second - first) + first


# Interpolate two motion points
def interpolate_motion_points(first, second, time):
    fraction = ((time - first.header.stamp) /
                (second.header.stamp - first.header.stamp))

    result = MotionPointStamped()
    result.header.stamp = time

    result.motion_point.pose.position.x = interpolate(
        first.motion_point.pose.position.x,
        second.motion_point.pose.position.x, fraction)
    result.motion_point.pose.position.y = interpolate(
        first.motion_point.pose.position.y,
        second.motion_point.pose.position.y, fraction)
    result.motion_point.pose.position.z = interpolate(
        first.motion_point.pose.position.z,
        second.motion_point.pose.position.z, fraction)

    result.motion_point.twist.linear.x = interpolate(
        first.motion_point.twist.linear.x, second.motion_point.twist.linear.x,
        fraction)
    result.motion_point.twist.linear.y = interpolate(
        first.motion_point.twist.linear.y, second.motion_point.twist.linear.y,
        fraction)
    result.motion_point.twist.linear.z = interpolate(
        first.motion_point.twist.linear.z, second.motion_point.twist.linear.z,
        fraction)

    result.motion_point.accel.linear.x = interpolate(
        first.motion_point.accel.linear.x, second.motion_point.accel.linear.x,
        fraction)
    result.motion_point.accel.linear.y = interpolate(
        first.motion_point.accel.linear.y, second.motion_point.accel.linear.y,
        fraction)
    result.motion_point.accel.linear.z = interpolate(
        first.motion_point.accel.linear.z, second.motion_point.accel.linear.z,
        fraction)

    return result

# Generates fully defined motion profiles
class LinearMotionProfileGenerator(object):
    def __init__(self, start_motion_point):
        self._last_motion_plan = MotionPointStampedArray()
        self._last_motion_plan.motion_points = [start_motion_point]

        try:
            self._TARGET_ACCEL = rospy.get_param(
                '~linear_motion_profile_acceleration')
            self._MAX_TARGET_ACCEL = rospy.get_param(
                '~linear_motion_profile_max_acceleration')
            self._PLAN_DURATION = rospy.get_param(
                '~linear_motion_profile_duration')
            self._PROFILE_TIMESTEP = rospy.get_param(
                '~linear_motion_profile_timestep')
        except KeyError as e:
            rospy.logerr(
                'Could not lookup a parameter for linear motion profile generator'
            )
            raise

        self._last_stamp = rospy.Time.now()

        self._override_start_position = None
        self._override_start_velocity = None

        # self._info = []

    linear_motion_profile_generator = None

    @staticmethod
    def get_linear_motion_profile_generator():
        if LinearMotionProfileGenerator.linear_motion_profile_generator is None:
            LinearMotionProfileGenerator.linear_motion_profile_generator = LinearMotionProfileGenerator(
                MotionPointStamped())
        return LinearMotionProfileGenerator.linear_motion_profile_generator

    # def dump_info(self):
    #     name = '/home/andrew/new_data.bin'
    #     file = open(name, 'wb')
    #     pickle.dump(self._info, file)
    #     file.close()

    def set_global_plan(self, plan):
        if len(self._last_motion_plan.motion_points) == 0:
            # self._info.append((self._last_motion_plan, plan))
            self._last_motion_plan = plan
        else:
            new_plan = MotionPointStampedArray()
            new_point_stamp = plan.motion_points[0].header.stamp

            for i in range(0, len(self._last_motion_plan.motion_points)):
                current_stamp = self._last_motion_plan.motion_points[i].header.stamp
                if current_stamp > new_point_stamp:
                    new_plan.motion_points = self._last_motion_plan.motion_points[:i]
                    break

            new_plan.motion_points.extend(plan.motion_points)
            # self._info.append((self._last_motion_plan, plan))

            self._last_motion_plan = new_plan

    # Get a starting motion point for a given time
    def _get_start_point(self,
                         time,
                         override_start_position,
                         override_start_velocity):

        start_motion_point = self.expected_point_at_time(time)

        # Apply a saved off overridden start point first
        if self._override_start_position is not None:
            if self._override_start_position.x is not None:
                start_motion_point.motion_point.pose.position.x = self._override_start_position.x
            if self._override_start_position.y is not None:
                start_motion_point.motion_point.pose.position.y = self._override_start_position.y
            if self._override_start_position.z is not None:
                start_motion_point.motion_point.pose.position.z = self._override_start_position.z
            self._override_start_position = None

        if self._override_start_velocity is not None:
            if self._override_start_velocity.x is not None:
                start_motion_point.motion_point.twist.linear.x = self._override_start_velocity.x
            if self._override_start_velocity.y is not None:
                start_motion_point.motion_point.twist.linear.y = self._override_start_velocity.y
            if self._override_start_velocity.z is not None:
                start_motion_point.motion_point.twist.linear.z = self._override_start_velocity.z
            self._override_start_velocity = None

        # Apply the passed in overriden start point second
        if override_start_position.x is not None:
            start_motion_point.motion_point.pose.position.x = override_start_position.x
        if override_start_position.y is not None:
            start_motion_point.motion_point.pose.position.y = override_start_position.y
        if override_start_position.z is not None:
            start_motion_point.motion_point.pose.position.z = override_start_position.z

        if override_start_velocity.x is not None:
            start_motion_point.motion_point.twist.linear.x = override_start_velocity.x
        if override_start_velocity.y is not None:
            start_motion_point.motion_point.twist.linear.y = override_start_velocity.y
        if override_start_velocity.z is not None:
            start_motion_point.motion_point.twist.linear.z = override_start_velocity.z

        return start_motion_point

    def set_start_point(self, start_point_command):
        self._override_start_position = start_point_command.start_position
        self._override_start_velocity = start_point_command.start_velocity

    def expected_point_at_time(self, time):
        # Make sure a starting point newer than the last sent time is sent
        for i in range(1, len(self._last_motion_plan.motion_points)):
            if self._last_motion_plan.motion_points[i].header.stamp > time:
                first_point = self._last_motion_plan.motion_points[i - 1]
                second_point = self._last_motion_plan.motion_points[i]
                return interpolate_motion_points(first_point, second_point,
                                                 time)
        # A point was not sent before the buffer ran out
        # Use the oldest and reset the timestamp
        self._last_motion_plan.motion_points[
            -1].header.stamp = rospy.Time.now()
        return self._last_motion_plan.motion_points[-1]

    # Get a motion plan that attempts to achieve a given velocity target
    def get_velocity_plan(self, velocity_command):

        # Get the stating motion point for the time that the velocity is desired
        start_point = self._get_start_point(
            velocity_command.target_twist.header.stamp,
            velocity_command.start_position,
            velocity_command.start_velocity)

        self._last_stamp = velocity_command.target_twist.header.stamp

        p_start = msg_to_np(start_point.motion_point.pose.position)
        v_start = msg_to_np(start_point.motion_point.twist.linear)
        v_desired = msg_to_np(velocity_command.target_twist.twist.linear)
        v_delta = v_desired - v_start


        acceleration = self._TARGET_ACCEL if velocity_command.acceleration is None \
                                          else velocity_command.acceleration

        if acceleration > self._MAX_TARGET_ACCEL:
            acceleration = self._MAX_TARGET_ACCEL
            rospy.logerr('iarc7_motion: linear motion profile generator \
                          requested acceleration is too large. \
                          Requested: {} Limit {}'.format(acceleration,
                                                         self._MAX_TARGET_ACCEL))

        # Assign a direction to the target acceleration
        a_target = acceleration * v_delta / np.linalg.norm(v_delta)

        # Find the time required to accelerate to the desired velocity
        acceleration_time = min(
            np.linalg.norm(v_delta) / acceleration, self._PLAN_DURATION)

        # Use the rest of the profile duration to hold the velocity
        steady_velocity_time = self._PLAN_DURATION - acceleration_time

        # Calculate the number of discrete steps spent in each state
        accel_steps = np.floor(acceleration_time / self._PROFILE_TIMESTEP)
        vel_steps = np.floor(steady_velocity_time / self._PROFILE_TIMESTEP)

        # Initialize the velocities array with the starting velocity
        velocities = []

        # Generate velocities corresponding to the acceleration period
        if accel_steps > 0:
            accel_times = np.linspace(
                0.0, accel_steps * self._PROFILE_TIMESTEP, accel_steps + 1)
            velocities.extend([
                a_target * accel_time + v_start for accel_time in accel_times
            ])

        # Add the velocities for the steady velocity period
        velocities.extend([v_desired for i in range(0, int(vel_steps))])

        velocities = np.array(velocities)

        # Differentiate velocities to get the accelerations
        # This results in an array one element shorter than
        # the velocities array
        accelerations = np.diff(velocities, axis=0) / self._PROFILE_TIMESTEP

        for i in range(0, accelerations.shape[0]):
            if np.linalg.norm(accelerations[i]) > 1.01 * acceleration:
                rospy.logerr(
                    'Linear Motion Profile generator produced an acceleration greater than the maximum allowed'
                )
                rospy.logerr('i {} accel steps {} vel steps {}'.format(
                    i, accel_steps, vel_steps))
                rospy.logerr('accel step times {}'.format(accel_times))
                rospy.logerr('v_d {} v_s {} a_t {}'.format(
                    v_desired, v_start, a_target))
                rospy.logerr('acceleration time {} v time {}'.format(
                    acceleration_time, steady_velocity_time))
                rospy.logerr('acceleration {} velocity {} {}'.format(
                    accelerations[i], velocities[i], velocities[i + 1]))

        # Integrate the velocities to get the position deltas
        # This results in an array the same length as the velocities array
        # But the positions correspond to the velocities one index higher than
        # the given positio index
        positions = (
            np.cumsum(velocities, axis=0) * self._PROFILE_TIMESTEP + p_start)

        plan = MotionPointStampedArray()
        pose_only_plan = Path()
        pose_only_plan.header.stamp = rospy.Time.now()
        pose_only_plan.header.frame_id = 'map'

        # Fill out the first motion point since it follows different
        # rules than the rest
        motion_point = MotionPointStamped()
        motion_point.header.stamp = start_point.header.stamp
        np_to_msg(accelerations[0], motion_point.motion_point.accel.linear)
        np_to_msg(v_start, motion_point.motion_point.twist.linear)
        np_to_msg(p_start, motion_point.motion_point.pose.position)

        plan.motion_points.append(motion_point)
        pose = PoseStamped()
        pose.header.stamp = motion_point.header.stamp
        pose.pose = motion_point.motion_point.pose
        pose_only_plan.poses.append(pose)

        # Generate the motion profile for all the remaining velocities and accelerations
        for i in range(1, velocities.shape[0] - 1):
            motion_point = MotionPointStamped()
            motion_point.header.stamp = (
                start_point.header.stamp +
                rospy.Duration(i * self._PROFILE_TIMESTEP))
            np_to_msg(accelerations[i], motion_point.motion_point.accel.linear)
            np_to_msg(velocities[i], motion_point.motion_point.twist.linear)
            np_to_msg(positions[i - 1],
                      motion_point.motion_point.pose.position)
            plan.motion_points.append(motion_point)
            pose = PoseStamped()
            pose.header.stamp = motion_point.header.stamp
            pose.pose = motion_point.motion_point.pose
            pose_only_plan.poses.append(pose)

        self._last_motion_plan = plan
        return plan, pose_only_plan

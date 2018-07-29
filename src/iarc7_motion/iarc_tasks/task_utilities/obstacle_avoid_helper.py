#!/usr/bin/env python
import rospy
import threading
import tf2_geometry_msgs
import tf2_ros
import numpy as np

from math import sin, cos, atan2, pi

from iarc7_safety.iarc_safety_exception import IARCSafetyException, IARCFatalSafetyException
from iarc7_msgs.msg import ObstacleArray
from geometry_msgs.msg import PointStamped

class ObstacleAvoider(object):
    def __init__(self, tf_buffer):
        self._lock = threading.RLock()

        self._avoid_distance = rospy.get_param('~obst_avoid_avoid_distance')
        self._predict_time = rospy.get_param('~obst_avoid_predict_time')
        self._response_strength = rospy.get_param('~obst_avoid_response_strength')

        # Maximum allowed distance from an obstacle to its projection on the nearest flight vector
        self._unsafe_obstacle_threshold = rospy.get_param("~obst_avoid_norm_limit")
        # Obstacles outside this radius are ignored
        self._obstacle_radius = rospy.get_param("~obst_avoid_radius")
        # Step size used when searching for alternative vectors when the requested vector is unsafe
        self._vector_step_size = rospy.get_param("~obst_avoid_step_size") # Divide by 180 to get degrees/step
        # Minimum avoid vector
        self._minimum_magnitude = rospy.get_param("~obst_avoid_min_magnitude")
        # Blending speed
        self._blending_speed = rospy.get_param('~obst_avoid_blending_speed')
        # Transform timeout
        self._timeout = rospy.Duration(rospy.get_param("~transform_timeout"))
        self._obstacle_points = None

        self._tf_buffer = tf_buffer

        with self._lock:
            self._obstacle_subscriber = rospy.Subscriber("/obstacles", ObstacleArray, self._update_obstacles)

    def wait_until_ready(self, startup_timeout):
        while (rospy.Time.now() == rospy.Time(0) and not rospy.is_shutdown()):
            # Wait for time to be initialized
            rospy.sleep(0.005)
        if rospy.is_shutdown():
            raise rospy.ROSInterruptException()

        start_time = rospy.Time.now()

        while ((self._obstacle_points is None)
               and rospy.Time.now() < start_time + startup_timeout
               and not rospy.is_shutdown()):
            rospy.sleep(0.005)
        if rospy.Time.now() >= start_time + startup_timeout:
            raise IARCFatalSafetyException('SafetyMonitor timed out on startup')
        if rospy.is_shutdown():
            raise rospy.ROSInterruptException()

    def _update_obstacles(self, obstacles):
        with self._lock:
            self._obstacle_points = []
            try:
                # Convert each obstacle into a numpy vector from the quad frame
                transform = self._tf_buffer.lookup_transform('level_quad', obstacles.header.frame_id, obstacles.header.stamp, self._timeout)
                for obstacle in obstacles.obstacles:
                    obstacle_point = PointStamped(header=obstacle.odom.header, point=obstacle.odom.pose.pose.position)
                    obstacle_point = tf2_geometry_msgs.do_transform_point(obstacle_point, transform)
                    obstacle_point = np.array([obstacle_point.point.x, obstacle_point.point.y])
                    # Ignore any obstacle out of the consideration radius
                    if np.linalg.norm(obstacle_point) <= self._obstacle_radius:
                        self._obstacle_points.append(obstacle_point)
            except (tf2_ros.LookupException,
                    tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException) as ex:
                rospy.logwarn("ObstacleAvoider: Couldn't lookup transform from {} to level_quad".format(obstacles.header.frame_id))

    def get_safe_vector(self, desired_vector, curr_vel):
        # Find the norm and direction of the velocity in the horizontal plane
        #original_vector_magnitude = np.linalg.norm(desired_vector[:2])
        #original_vector_direction = atan2(desired_vector[1], desired_vector[0])
        curr_vel = np.array(curr_vel)
        desired_vector = np.array(desired_vector)
        with self._lock:
            for obstacle in self._obstacle_points:
                if np.linalg.norm(obstacle) == 0:
                    unit_vector = np.array([1, 0, 0])
                else:
                    unit_vector = obstacle / np.linalg.norm(obstacle)
                away_unit_vector = -unit_vector

                turn_dir = np.array([0, 0, np.cross(obstacle, curr_vel)])
                if np.linalg.norm(turn_dir) < 1e-6:
                    turn_dir = np.array([0, 0, 1])

                side_vector = np.cross(turn_dir, obstacle)
                unit_side = side_vector / np.linalg.norm(side_vector)

                rel_vel = np.dot(unit_vector, curr_vel)
                violation = max(0,
                                self._avoid_distance
                              - np.linalg.norm(obstacle)
                              + rel_vel * self._predict_time)

                # Push away
                desired_vector[:2] += away_unit_vector \
                        * self._response_strength * violation**2

                # Push around
                desired_vector += unit_side * violation * 0.3
            return desired_vector
            #for angle in np.linspace(0, pi, num=self._vector_step_size):
            #    for sign in [-1, 1]:
            #        vector_is_safe = True
            #        new_vector = np.array([cos(original_vector_direction+angle*sign), sin(original_vector_direction+angle*sign)])
            #        for obstacle in self._obstacle_points:
            #            # Project the obstacle vector onto the flight vector
            #            obstacle_proj_new_vector = (np.dot(new_vector, obstacle)/np.dot(new_vector, new_vector))*new_vector

            #            # Don't look at obstacles projected onto the reflection of this vector
            #            if np.sign(obstacle_proj_new_vector[0]) == np.sign(new_vector[0]) and np.sign(obstacle_proj_new_vector[1]) == np.sign(new_vector[1]):
            #                # find distance of the obstacle to its projection on the flight vector
            #                obstacle_to_vector_distance = np.linalg.norm(obstacle - obstacle_proj_new_vector)
            #                # throw out vectors with obstacles that are too close
            #                if obstacle_to_vector_distance <= self._unsafe_obstacle_threshold:
            #                    vector_is_safe = False
            #                    break

            #        # If this flight vector is safe, go for it
            #        if vector_is_safe:
            #            if angle != 0:
            #                rospy.logwarn("ObstacleAvoider: modified requested velocity by {} radians".format(angle))
            #                # We need to avoid, so make sure the vector magnitude is non-zero
            #                response_magnitude = max(original_vector_magnitude, self._minimum_magnitude)

            #            # Copy Z velocity from commanded vector
            #            # New flight vector has same magnitude as the commanded
            #            barely_safe_vel = np.array([
            #                response_magnitude
            #              * cos(original_vector_direction+angle*sign),
            #                response_magnitude
            #              * sin(original_vector_direction+angle*sign),
            #                desired_vector[2]])
            #            reflected_safe_vel = np.array([
            #                response_magnitude
            #              * cos(original_vector_direction+2*angle*sign),
            #                response_magnitude
            #              * sin(original_vector_direction+2*angle*sign),
            #                desired_vector[2]])
            #            blend_constant = np.exp(-original_vector_magnitude / self._blending_speed)
            #            return barely_safe_vel * blend_constant + reflected_safe_vel * (1-blend_constant)

        #rospy.logwarn("ObstacleAvoider: couldn't find safe velocity!")
        ## Couldn't find a safe vector, just stop
        #return np.array([0, 0, desired_vector[2]])


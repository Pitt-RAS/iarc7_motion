#!/usr/bin/env python
import rospy
import threading
import tf2_geometry_msgs
import tf2_ros
import numpy as np

from math import sin, cos, atan2, pi

from iarc7_safety.iarc_safety_exception import IARCSafetyException
from iarc7_msgs.msg import ObstacleArray
from geometry_msgs.msg import PointStamped

class IdleObstacleAvoider(object):
    def __init__(self):
        self._lock = threading.RLock()

        # Maximum allowed distance from an obstacle to its projection on the nearest flight vector
        # Obstacles with norms greater than this are not considered obstacles
        self._unsafe_obstacle_threshold = rospy.get_param("~obst_avoid_norm_limit")
        # Obstacles outside this radius are ignored
        self._obstacle_radius = rospy.get_param("~obst_avoid_radius")
        # Step size used when searching for alternative vectors when the requested vector is unsafe
        self._vector_step_size = rospy.get_param("~obst_avoid_step_size")
        # Transform timeout
        self._timeout = rospy.Duration(rospy.get_param("~transform_timeout"))
        self._obstacle_points = None

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

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
                rospy.logwarn("ObstacleAvoider Couldn't lookup transform from {} to level_quad".format(obstacles.header.frame_id))

    def get_safest(self, magnitude):
        with self._lock:
            regions = []
            startOpenRegion = None
            for angle in np.linspace(0, 2*pi, num=self._vector_step_size*2):
                vector_is_safe = True
                new_vector = np.array([cos(angle), sin(angle)])
                for obstacle in self._obstacle_points:
                    # Project the obstacle vector onto the flight vector
                    obstacle_proj_new_vector = (np.dot(new_vector, obstacle)/np.dot(new_vector, new_vector))*new_vector

                    # Don't look at obstacles projected onto the reflection of this vector
                    if np.sign(obstacle_proj_new_vector[0]) == np.sign(new_vector[0]) and np.sign(obstacle_proj_new_vector[1]) == np.sign(new_vector[1]):
                        # find distance of the obstacle to its projection on the flight vector
                        obstacle_to_vector_distance = np.linalg.norm(obstacle - obstacle_proj_new_vector)
                        # throw out vectors with obstacles that are too close
                        if obstacle_to_vector_distance <= self._unsafe_obstacle_threshold:
                            vector_is_safe = False
                            break

                if vector_is_safe and startOpenRegion is None:
                    startOpenRegion = angle
                elif not vector_is_safe and startOpenRegion is not None:
                    regions.append([startOpenRegion, angle])
                    startOpenRegion = None

            # Handle ending on a safe region
            if startOpenRegion is not None:
                lastRegionSize = 2*pi - startOpenRegion
                # Check if we can combine [a, 2*pi] and [0, b] to be [a, b]
                if len(regions) is not 0 and regions[0][0] is 0:
                    regions[0][0] = -lastRegionSize
                # Can't combine with start region
                else:
                    regions.append([startOpenRegion, 2*pi])

            # Find the largest range of open space and head in that direction
            biggestRegion = 0
            region = None
            for r in regions:
                if r[1]-r[0] > biggestRegion:
                    biggestRegion = r[1]-r[0]
                    region = r

            # No regions are safe
            if region is None:
                rospy.logwarn("IdleObstacleAvoider couldn't find safe velocity!")
                return np.array([0,0,0])
            # All regions are safe
            elif np.isclose(region[0], 0) and np.isclose(region[1], 2*pi):
                return np.array([0,0,0])
            # Fly the vector we found
            else:
                vectorAngle = region[0] + (region[1]-region[0])/2.0
                return np.array([magnitude * cos(vectorAngle), magnitude * sin(vectorAngle), 0])


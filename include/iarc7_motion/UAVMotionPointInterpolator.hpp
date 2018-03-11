////////////////////////////////////////////////////////////////////////////
//
// UAV Motion Point Interpolator
//
// Stores a queue of UAV Motion Points commands from a topic. Interpolates
// between points when a UAV Motion Point is requested.
//
////////////////////////////////////////////////////////////////////////////

#ifndef UAV_MOTION_POINT_INTERPOLATOR_H
#define UAV_MOTION_POINT_INTERPOLATOR_H

#include <ros/ros.h>
#include <vector>

#include "gtest/gtest_prod.h"

#include "iarc7_msgs/UAVMotionPointStampedArray.h"
#include "iarc7_msgs/UAVMotionPoint.h"

using iarc7_msgs::UAVMotionPointStamped;
using geometry_msgs::Pose;
using geometry_msgs::Twist;
using geometry_msgs::Accel;

namespace Iarc7Motion
{

// Typdef the vector template to a shorter name for ease of typing
typedef std::vector<UAVMotionPointStamped> UAVMotionPointStampedArray;

class UAVMotionPointInterpolator
{
public:
    UAVMotionPointInterpolator() = delete;

    // Require construction with a node handle
    UAVMotionPointInterpolator(ros::NodeHandle& nh);

    ~UAVMotionPointInterpolator() = default;

    // Don't allow the copy constructor or assignment.
    UAVMotionPointInterpolator(const UAVMotionPointInterpolator& rhs) = delete;
    UAVMotionPointInterpolator& operator=(const UAVMotionPointInterpolator& rhs) = delete;

    // Called by a class user to get a target motion point
    // Makes the call to trim the queue during the class
    // Interpolates if there is more than one twist
    void getTargetMotionPoint(
                    const ros::Time& current_time,
                    UAVMotionPointStamped& target_uav_motion_point);

private:
    // Used to allow unit tests to call private functions
    FRIEND_TEST(UAVMotionPointInterpolatorTests, testTrimMotionPointQueue);
    FRIEND_TEST(UAVMotionPointInterpolatorTests, testAppendMotionPointQueue);

    // Receive a new list of velocities commands and call appendVelocityQueue to insert them into the queue
    void processMotionPointArray(
        const iarc7_msgs::UAVMotionPointStampedArray::ConstPtr& message);

    // Trim the velocity queue so that there aren't old velocities in the queue
    static bool trimMotionPointQueue(
        UAVMotionPointStampedArray& motion_points,
        const ros::Time& time);

    // Append velocity targets to the velocity queue. If the velocity targets are older than the newest velocity target queued
    // the queued velocity targets are discarded
    static bool appendMotionPointQueue(
        UAVMotionPointStampedArray& current_motion_points,
        const UAVMotionPointStampedArray& new_motion_points,
        const ros::Time& time);

    // Takes two twists and interpolates between their values
    // using linear interpolation
    static bool interpolateMotionPoints(UAVMotionPointStamped& begin,
                                        UAVMotionPointStamped& end,
                                        UAVMotionPointStamped& interpolated,
                                        ros::Time time);

    // Interpolate between arbitrary values and waypoints
    // x is to be between 0-1 if the interpolation is to
    // range from start to end.
    static double interpolate(double x, double start, double end);

    // Subscriber for uav motion point targets
    ros::Subscriber uav_motion_points_subscriber_;

    // Queue of motion points with timestamps.
    UAVMotionPointStampedArray uav_motion_point_targets_;
};

} // End namespace Iarc7Motion

#endif // UAV_MOTION_POINT_INTERPOLATOR_H

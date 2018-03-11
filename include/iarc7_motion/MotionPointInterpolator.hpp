////////////////////////////////////////////////////////////////////////////
//
// Motion Point Interpolator
//
// Stores a queue of Motion Points commands from a topic. Interpolates
// between points when a Motion Point is requested.
//
////////////////////////////////////////////////////////////////////////////

#ifndef MOTION_POINT_INTERPOLATOR_H
#define MOTION_POINT_INTERPOLATOR_H

#include <ros/ros.h>
#include <vector>

#include "gtest/gtest_prod.h"

#include "iarc7_msgs/MotionPointStampedArray.h"
#include "iarc7_msgs/MotionPoint.h"

using iarc7_msgs::MotionPointStamped;
using geometry_msgs::Pose;
using geometry_msgs::Twist;
using geometry_msgs::Accel;

namespace Iarc7Motion
{

// Typdef the vector template to a shorter name for ease of typing
typedef std::vector<MotionPointStamped> MotionPointStampedArray;

class MotionPointInterpolator
{
public:
    MotionPointInterpolator() = delete;

    // Require construction with a node handle
    MotionPointInterpolator(ros::NodeHandle& nh);

    ~MotionPointInterpolator() = default;

    // Don't allow the copy constructor or assignment.
    MotionPointInterpolator(const MotionPointInterpolator& rhs) = delete;
    MotionPointInterpolator& operator=(const MotionPointInterpolator& rhs) = delete;

    // Called by a class user to get a target motion point
    // Makes the call to trim the queue during the class
    // Interpolates if there is more than one twist
    void getTargetMotionPoint(
                    const ros::Time& current_time,
                    MotionPointStamped& target_motion_point);

private:
    // Used to allow unit tests to call private functions
    FRIEND_TEST(MotionPointInterpolatorTests, testTrimMotionPointQueue);
    FRIEND_TEST(MotionPointInterpolatorTests, testAppendMotionPointQueue);

    // Receive a new list of velocities commands and call appendVelocityQueue to insert them into the queue
    void processMotionPointArray(
        const iarc7_msgs::MotionPointStampedArray::ConstPtr& message);

    // Trim the velocity queue so that there aren't old velocities in the queue
    static bool trimMotionPointQueue(
        MotionPointStampedArray& motion_points,
        const ros::Time& time);

    // Append velocity targets to the velocity queue. If the velocity targets are older than the newest velocity target queued
    // the queued velocity targets are discarded
    static bool appendMotionPointQueue(
        MotionPointStampedArray& current_motion_points,
        const MotionPointStampedArray& new_motion_points,
        const ros::Time& time);

    // Takes two twists and interpolates between their values
    // using linear interpolation
    static bool interpolateMotionPoints(MotionPointStamped& begin,
                                        MotionPointStamped& end,
                                        MotionPointStamped& interpolated,
                                        ros::Time time);

    // Interpolate between arbitrary values and waypoints
    // x is to be between 0-1 if the interpolation is to
    // range from start to end.
    static double interpolate(double x, double start, double end);

    // Subscriber for motion point targets
    ros::Subscriber motion_points_subscriber_;

    // Queue of motion points with timestamps.
    MotionPointStampedArray motion_point_targets_;
};

} // End namespace Iarc7Motion

#endif // MOTION_POINT_INTERPOLATOR_H

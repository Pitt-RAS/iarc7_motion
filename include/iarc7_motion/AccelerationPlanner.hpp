////////////////////////////////////////////////////////////////////////////
//
// Acceleration Planner
//
// Stores a queue of velocity commands from a topic. Interpolates
// between velocities when a velocity is requested. Allows for smooth
// acceleration.
//
////////////////////////////////////////////////////////////////////////////

#ifndef ACCELERATION_PLANNER_H
#define ACCELERATION_PLANNER_H

#include <ros/ros.h>
#include <vector>

#include "gtest/gtest_prod.h"

#include "iarc7_msgs/TwistStampedArray.h"
#include "geometry_msgs/TwistStamped.h"

using geometry_msgs::TwistStamped;

namespace Iarc7Motion
{

// Typdef the vector template to a shorter name for ease of typing
typedef std::vector<geometry_msgs::TwistStamped> TwistStampedArray;

class AccelerationPlanner
{
public:
    AccelerationPlanner() = delete;

    // Require construction with a node handle
    AccelerationPlanner(ros::NodeHandle& nh);

    ~AccelerationPlanner() = default;

    // Don't allow the copy constructor or assignment.
    AccelerationPlanner(const AccelerationPlanner& rhs) = delete;
    AccelerationPlanner& operator=(const AccelerationPlanner& rhs) = delete;

    // Used to get a twist. If needed the twist is interpolated between a velocity target 
    // that occurs prior to the requested time and after the requested time
    void getTargetTwist(const ros::Time& current_time, geometry_msgs::TwistStamped& target_twist);

private:
    // Used to allow unit tests to call private functions
    FRIEND_TEST(AccelerationPlannerTests, testTrimVelocityQueue);
    FRIEND_TEST(AccelerationPlannerTests, testAppendVelocityQueue);

    // Handles incoming velocity command messages
    void processVelocityCommand(const iarc7_msgs::TwistStampedArray::ConstPtr& message);

    // Removes old velocities from the velocity queue
    static bool trimVelocityQueue(TwistStampedArray& twists, const ros::Time& time);

    // Appends new velocities to the velocity queue, overwrites velocities in the  velocity queue if appropriate
    static bool appendVelocityQueue(TwistStampedArray& current_twists, const TwistStampedArray& new_twists, const ros::Time& time);

    // Interpolates between two twists according ax+b
    static bool interpolateTwists(TwistStamped& begin, TwistStamped& end, TwistStamped& interpolated, ros::Time time);

    // Subscriber for uav_arm
    ros::Subscriber velocity_targets_subscriber_;

    // Queue of velocities with timestamps. A velocity with a timestamp is considered
    // a velocity target.
    TwistStampedArray velocity_targets_;
};

} // End namespace Iarc7Motion

#endif // ACCELERATION_PLANNER_H

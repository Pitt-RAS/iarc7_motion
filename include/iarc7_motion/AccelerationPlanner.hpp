////////////////////////////////////////////////////////////////////////////
//
// Acceleration Planner
//
// Implements receiving twist arrays and the acceleration ramps
//
////////////////////////////////////////////////////////////////////////////

#ifndef ACCELERATION_PLANNER_H
#define ACCELERATION_PLANNER_H

#include <ros/ros.h>
#include <vector>

#include "gtest/gtest_prod.h"

#include "iarc7_msgs/TwistStampedArrayStamped.h"
#include "geometry_msgs/TwistStamped.h"

using geometry_msgs::TwistStamped;

namespace Iarc7Motion
{

typedef std::vector<geometry_msgs::TwistStamped> TwistStampedArray;

class AccelerationPlanner
{
public:
    AccelerationPlanner() = delete;

    AccelerationPlanner(ros::NodeHandle& nh);

    ~AccelerationPlanner() = default;

    // Don't allow the copy constructor or assignment.
    AccelerationPlanner(const AccelerationPlanner& rhs) = delete;
    AccelerationPlanner& operator=(const AccelerationPlanner& rhs) = delete;

    void update();

    void getTargetTwist(const ros::Time& current_time, geometry_msgs::TwistStamped& target_twist);

private:
    FRIEND_TEST(AccelerationPlannerTests, testTrimVelocityQueue);
    FRIEND_TEST(AccelerationPlannerTests, testAppendVelocityQueue);

    void processVelocityCommand(const iarc7_msgs::TwistStampedArrayStamped::ConstPtr& message);

    static bool trimVelocityQueue(TwistStampedArray& twists, const ros::Time& time);

    static bool appendVelocityQueue(TwistStampedArray& current_twists, const TwistStampedArray& new_twists, const ros::Time& time);

    static bool interpolateTwists(TwistStamped& begin, TwistStamped& end, TwistStamped& interpolated, ros::Time time);

    ros::NodeHandle& nh_;

    // Subscriber for uav_arm
    ros::Subscriber velocity_targets_subscriber_;

    TwistStampedArray velocity_targets_;
};

} // End namespace Iarc7Motion

#endif // ACCELERATION_PLANNER_H

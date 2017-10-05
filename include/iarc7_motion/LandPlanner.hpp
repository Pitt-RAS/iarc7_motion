////////////////////////////////////////////////////////////////////////////
//
// Land Controller
//
// Handles basic landing
//
////////////////////////////////////////////////////////////////////////////

#ifndef LAND_CONTROLLER_H
#define LAND_CONTROLLER_H

#include <ros/ros.h>

#include "ros_utils/LinearMsgInterpolator.hpp"
#include "ros_utils/SafeTransformWrapper.hpp"

// ROS message headers
#include "geometry_msgs/TwistStamped.h"
#include "iarc7_msgs/LandingGearContactsStamped.h"
#include "iarc7_msgs/Arm.h"

namespace Iarc7Motion
{

enum class LandState { ACCELERATE_TO_DESCENT_VELOCITY,
                       DESCEND,
                       BRACE_FOR_IMPACT,
                       DISARM,
                       DONE };

class LandPlanner
{
public:
    LandPlanner() = delete;

    // Require construction with a node handle and action server
    LandPlanner(ros::NodeHandle& nh, ros::NodeHandle& private_nh);

    ~LandPlanner() = default;

    // Don't allow the copy constructor or assignment.
    LandPlanner(const LandPlanner& rhs) = delete;
    LandPlanner& operator=(const LandPlanner& rhs) = delete;

    // Used to prepare and check initial conditions for landing
    bool __attribute__((warn_unused_result)) prepareForTakeover(
        const ros::Time& time);

    // Used to get a uav control message
    bool __attribute__((warn_unused_result)) getTargetTwist(
        const ros::Time& time,
        geometry_msgs::TwistStamped& target_twist);

    /// Waits until this object is ready to begin normal operation
    bool __attribute__((warn_unused_result)) waitUntilReady();

    bool isDone();

private:
    // Handles incoming landing gear messages
    void processLandingGearMessage(
        const iarc7_msgs::LandingGearContactsStamped::ConstPtr& message);

    static bool allPressed(const iarc7_msgs::LandingGearContactsStamped& msg);
    static bool anyPressed(const iarc7_msgs::LandingGearContactsStamped& msg);

    iarc7_msgs::LandingGearContactsStamped landing_gear_message_;

    ros::Subscriber landing_gear_subscriber_;

    bool landing_gear_message_received_;

    ros_utils::SafeTransformWrapper transform_wrapper_;

    LandState state_;

    double requested_z_vel_;

    // Rate at which to accelerate from 0 to descent velocity
    const double descend_acceleration_;

    // Rate at whicch to descent
    const double descend_rate_;

    // How hard to brace
    const double brace_impact_velocity_;

    // When to begin bracing
    const double brace_impact_start_height_;

    // If we go above this height during brace we just report failure.
    // The land sequence didn't work.
    const double brace_impact_failure_height_;

    // This parameter will be obsolete when the foot touch sensors are implemented
    const double brace_impact_success_height_;

    // Last time an update was successful
    ros::Time last_update_time_;

    // Max allowed timeout waiting for first velocity and transform
    const ros::Duration startup_timeout_;

    // Max allowed timeout waiting for velocities and transforms
    const ros::Duration update_timeout_;

    // Establishing service client used for disarm request
    ros::ServiceClient uav_arm_client_;
};

} // End namespace Iarc7Motion

#endif // LAND_CONTROLLER_H

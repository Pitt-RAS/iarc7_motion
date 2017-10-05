////////////////////////////////////////////////////////////////////////////
//
// Land Planner
//
// Handles landing
//
////////////////////////////////////////////////////////////////////////////

// Associated header
#include "iarc7_motion/LandPlanner.hpp"

// ROS Headers
#include "ros_utils/ParamUtils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

using namespace Iarc7Motion;

LandPlanner::LandPlanner(
        ros::NodeHandle& nh,
        ros::NodeHandle& private_nh)
    : transform_wrapper_(),
      state_(LandState::DONE),
      requested_z_vel_(0.0),
      descend_acceleration_(ros_utils::ParamUtils::getParam<double>(
              private_nh,
              "descend_acceleration")),
      descend_rate_(ros_utils::ParamUtils::getParam<double>(
              private_nh,
              "descend_rate")),
      brace_impact_velocity_(ros_utils::ParamUtils::getParam<double>(
              private_nh,
              "brace_impact_velocity")),
      brace_impact_start_height_(ros_utils::ParamUtils::getParam<double>(
              private_nh,
              "brace_impact_start_height")),
      brace_impact_failure_height_(ros_utils::ParamUtils::getParam<double>(
              private_nh,
              "brace_impact_failure_height")),
      brace_impact_success_height_(ros_utils::ParamUtils::getParam<double>(
              private_nh,
              "brace_impact_success_height")),
      last_update_time_(),
      startup_timeout_(ros_utils::ParamUtils::getParam<double>(
              private_nh,
              "startup_timeout")),
      update_timeout_(ros_utils::ParamUtils::getParam<double>(
              private_nh,
              "update_timeout")),
      uav_arm_client_()
{
    landing_gear_subscriber_ = nh.subscribe("landing_gear_contact_switches",
                                 100,
                                 &LandPlanner::processLandingGearMessage,
                                 this);
    //Setting up the service client connection for disarm request
    uav_arm_client_ = nh.serviceClient<iarc7_msgs::Arm>("uav_arm");
}

// Used to prepare and check initial conditions for landing
// getTargetTwist needs to begin being called shortly after this is called.
bool LandPlanner::prepareForTakeover(const ros::Time& time)
{
    if (time < last_update_time_) {
        ROS_ERROR("Tried to reset LandPlanner with time before last update");
        return false;
    }

    if(allPressed(landing_gear_message_)) {
        ROS_ERROR("Tried to reset the LandPlanner while being on the ground");
        return false;
    }

    requested_z_vel_ = 0;
    state_ = LandState::ACCELERATE_TO_DESCENT_VELOCITY;
    // Mark the last update time as the current time since update may not have
    // Been called in a long time.
    last_update_time_ = time;
    return true;
}

// Main update
bool LandPlanner::getTargetTwist(const ros::Time& time,
                         geometry_msgs::TwistStamped& target_twist)
{
    if (time < last_update_time_) {
        ROS_ERROR("Tried to update LandPlanner with time before last update");
        return false;
    }

    // Get the current transform (xyz) of the quad
    geometry_msgs::TransformStamped transform;
    bool success = transform_wrapper_.getTransformAtTime(transform,
                                                         "map",
                                                         "base_footprint",
                                                         time,
                                                         update_timeout_);
    if (!success) {
        ROS_ERROR("Failed to get current transform in LandPlanner::update");
        return false;
    }

    if(state_ == LandState::ACCELERATE_TO_DESCENT_VELOCITY) {
        requested_z_vel_ = std::max(requested_z_vel_ - ((time - last_update_time_).toSec() * descend_acceleration_), descend_rate_);
        if(requested_z_vel_ <= descend_rate_) {
            state_ = LandState::DESCEND;
        }
    }
    else if(state_ == LandState::DESCEND)
    {
        requested_z_vel_ = descend_rate_;
        // Continue to ascend when done return success and turn off
        if(transform.transform.translation.z < brace_impact_start_height_)
        {
            state_ = LandState::BRACE_FOR_IMPACT;
        }
    }
    else if(state_ == LandState::BRACE_FOR_IMPACT) {
        requested_z_vel_ = brace_impact_velocity_;
        if(transform.transform.translation.z > brace_impact_failure_height_) {
            ROS_ERROR("Land sequence failed, quad started going up again");
            return false;
        }
        else if(anyPressed(landing_gear_message_)) {
            state_ = LandState::DISARM;
        }
    }
    else if (state_ == LandState::DISARM) {
      //Sending disarm request to fc_comms
      iarc7_msgs::Arm srv;
      srv.request.data = false;
      //Check if request was succesful
      if(uav_arm_client_.call(srv)) {
        if(srv.response.success == false) {
          ROS_ERROR("Service could not disarm the controller");
          return false;
        }
      }
      else {
        ROS_ERROR("disarming service failed");
        return false;
      }
      state_ = LandState::DONE;
    }
    else if(state_ == LandState::DONE)
    {
        ROS_ERROR("Tried to update takeoff handler when in DONE state");
        return false;
    }
    else
    {
        ROS_ASSERT_MSG(false, "Invalid state in takeoff controller");
    }

    // Fill in the uav_command's information
    target_twist.header.stamp = time;

    target_twist.twist.linear.z = requested_z_vel_;

    last_update_time_ = time;
    return true;
}

bool LandPlanner::waitUntilReady()
{
    geometry_msgs::TransformStamped transform;
    bool success = transform_wrapper_.getTransformAtTime(transform,
                                                         "map",
                                                         "base_footprint",
                                                         ros::Time(0),
                                                         startup_timeout_);
    if (!success)
    {
        ROS_ERROR("Failed to fetch transform");
        return false;
    }

    const ros::Time start_time = ros::Time::now();
    while (ros::ok()
           && !landing_gear_message_received_
           && ros::Time::now() < start_time + startup_timeout_) {
        ros::spinOnce();
        ros::Duration(0.005).sleep();
    }

    if (!landing_gear_message_received_) {
        ROS_ERROR_STREAM("LandPlanner failed to fetch initial switch message");
        return false;
    }

    // This time is just used to calculate any ramping that needs to be done.
    last_update_time_ = landing_gear_message_.header.stamp;
    return true;
}

bool LandPlanner::isDone()
{
  return (state_ == LandState::DONE);
}

void LandPlanner::processLandingGearMessage(
    const iarc7_msgs::LandingGearContactsStamped::ConstPtr& message)
{
    landing_gear_message_received_ = true;
    landing_gear_message_ = *message;
}

bool LandPlanner::allPressed(const iarc7_msgs::LandingGearContactsStamped& msg)
{
    return msg.front && msg.back && msg.left && msg.right;
}

bool LandPlanner::anyPressed(const iarc7_msgs::LandingGearContactsStamped& msg)
{
    return msg.front || msg.back || msg.left || msg.right;
}

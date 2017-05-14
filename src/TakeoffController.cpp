////////////////////////////////////////////////////////////////////////////
//
// Takeoff Controller
//
// Handles takeoff
//
////////////////////////////////////////////////////////////////////////////

// Associated header
#include "iarc7_motion/TakeoffController.hpp"

// ROS Headers
#include "ros_utils/ParamUtils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

using namespace Iarc7Motion;

TakeoffController::TakeoffController(
        ros::NodeHandle& /*nh*/,
        ros::NodeHandle& private_nh,
        const ThrustModel& thrust_model)
    : transform_wrapper_(),
      state_(TakeoffState::DONE),
      throttle_(),
      thrust_model_(thrust_model),
      max_takeoff_start_height_(ros_utils::ParamUtils::getParam<double>(
              private_nh,
              "max_takeoff_start_height")),
      takeoff_throttle_ramp_rate_(ros_utils::ParamUtils::getParam<double>(
              private_nh,
              "takeoff_throttle_ramp_rate")),
      last_update_time_(),
      startup_timeout_(ros_utils::ParamUtils::getParam<double>(
              private_nh,
              "startup_timeout")),
      update_timeout_(ros_utils::ParamUtils::getParam<double>(
              private_nh,
              "update_timeout"))
{
}

// Used to reset and check initial conditions for takeoff
// Update needs to begin being called shortly after this is called.
bool TakeoffController::prepareForTakeover(const ros::Time& time)
{
    if (time < last_update_time_) {
        ROS_ERROR("Tried to reset TakeoffHandler with time before last update");
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
        ROS_ERROR("Failed to get current transform in TakeoffController::update");
        return false;
    }

    if(transform.transform.translation.z > max_takeoff_start_height_) {
        ROS_ERROR("Tried to reset the takeoff controller without being on the ground");
        return false;
    } else if (state_ != TakeoffState::DONE) {
        ROS_ERROR("Tried to reset takeoff controller that wasn't in the done state");
        return false;
    }

    throttle_ = 0;
    state_ = TakeoffState::RAMP;
    // Mark the last update time as the current time to prevent large throttle spikes
    last_update_time_ = time;
    return true;
}

// Main update
bool TakeoffController::update(const ros::Time& time,
                            iarc7_msgs::OrientationThrottleStamped& uav_command)
{
    if (time < last_update_time_) {
        ROS_ERROR("Tried to update TakeoffHandler with time before last update");
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
        ROS_ERROR("Failed to get current transform in TakeoffController::update");
        return false;
    }

    if(state_ == TakeoffState::RAMP) {
        if(transform.transform.translation.z > max_takeoff_start_height_) {
          // Do something to modify the thrust model
          state_ = TakeoffState::DONE;
        }
        else
        {
          throttle_ = std::min(throttle_ + ((time - last_update_time_).toSec() * takeoff_throttle_ramp_rate_), 1.0);
        }
    }
    else if(state_ == TakeoffState::DONE)
    {
        ROS_ERROR("Tried to update takeoff handler when in DONE state");
        return false;
    }
    else
    {
        ROS_ASSERT_MSG(false, "Invalid state in takeoff controller");
    }

    // Fill in the uav_command's information
    uav_command.header.stamp = time;

    uav_command.throttle = throttle_;

    // Check that none of the throttle values are infinite before returning
    if (!std::isfinite(uav_command.throttle)
     || !std::isfinite(uav_command.data.pitch)
     || !std::isfinite(uav_command.data.roll)
     || !std::isfinite(uav_command.data.yaw)) {
        ROS_ERROR(
            "Part of command is not finite in TakeoffHandler::update");
        return false;
    }

    // Print the throttle information
    ROS_DEBUG("Throttle: %f Pitch: %f Roll: %f Yaw: %f",
             uav_command.throttle,
             uav_command.data.pitch,
             uav_command.data.roll,
             uav_command.data.yaw);

    last_update_time_ = time;
    return true;
}

bool TakeoffController::waitUntilReady()
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

    // Mark the last update time the current time as we could not have
    // received a message before this
    // Note that this is not the same time as the transform.
    // We get the last transform off the stack just to make sure there is something there.
    // This time is just used to calculate any ramping that needs to be done.
    last_update_time_ = ros::Time::now();
    return true;
}

bool TakeoffController::isDone()
{
  return (state_ == TakeoffState::DONE);
}

ThrustModel TakeoffController::getThrustModel()
{
  return thrust_model_;
}

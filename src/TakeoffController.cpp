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
#include <ros/ros.h>
#include "ros_utils/ParamUtils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include <geometry_msgs/PointStamped.h>

using namespace Iarc7Motion;

TakeoffController::TakeoffController(
        ros::NodeHandle& nh,
        ros::NodeHandle& private_nh,
        const ThrustModel& thrust_model)
    : landing_detected_message_(),
      landing_detected_subscriber_(),
      landing_detected_message_received_(false),
      transform_wrapper_(),
      state_(TakeoffState::DONE),
      throttle_(),
      thrust_model_(thrust_model),
      takeoff_throttle_ramp_rate_(ros_utils::ParamUtils::getParam<double>(
              private_nh,
              "takeoff_throttle_ramp_rate")),
      last_update_time_(),
      startup_timeout_(ros_utils::ParamUtils::getParam<double>(
              private_nh,
              "startup_timeout")),
      update_timeout_(ros_utils::ParamUtils::getParam<double>(
              private_nh,
              "update_timeout")),
      battery_timeout_(ros_utils::ParamUtils::getParam<double>(
              private_nh,
              "battery_timeout")),
      battery_interpolator_(nh,
                            "motor_battery",
                            update_timeout_,
                            battery_timeout_,
                            [](const iarc7_msgs::Float64Stamped& msg) {
                                return msg.data;
                            },
                            100),
      takeoff_max_height_switch_pressed_(ros_utils::ParamUtils::getParam<double>(
                  private_nh,
                  "takeoff_max_height_switch_pressed")),
      uav_arm_client_(nh.serviceClient<iarc7_msgs::Arm>("uav_arm")),
      arm_time_(),
      ramp_start_time_()
{
    landing_detected_subscriber_ = nh.subscribe("landing_detected",
                                    10,
                                    &TakeoffController::processLandingDetectedMessage,
                                    this);
}

bool TakeoffController::calibrateThrustModel(const ros::Time& time)
{
    double voltage;
    if (!battery_interpolator_.getInterpolatedMsgAtTime(voltage, time)) {
        ROS_ERROR("Failed to get battery voltage to calibrate thrust model");
        return false;
    }

    geometry_msgs::TransformStamped transform;
    bool success = transform_wrapper_.getTransformAtTime(transform,
                                                         "map",
                                                         "center_of_lift",
                                                         time,
                                                         update_timeout_);
    if (!success) {
        ROS_ERROR("Failed to get current transform to calibrate thrust model");
        return false;
    }

    geometry_msgs::PointStamped col_point;
    tf2::doTransform(col_point, col_point, transform);

    thrust_model_.calibrate(0.57, voltage, col_point.point.z);
    return true;
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

    if(!landing_detected_message_.data) {
        ROS_ERROR("Tried to reset the takeoff controller without being on the ground");
        return false;
    } else if (state_ != TakeoffState::DONE) {
        ROS_ERROR("Tried to reset takeoff controller that wasn't in the done state");
        return false;
    }

    throttle_ = 0;
    state_ = TakeoffState::ARM;
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

    if(state_ == TakeoffState::ARM) {
        // Sending arm request to fc_comms
        iarc7_msgs::Arm srv;
        srv.request.data = true;
        // Check if request was succesful
        if(uav_arm_client_.call(srv)) {
            if(srv.response.success == false) {
                ROS_ERROR("Service could not arm the controller");
                return false;
            }
        }
        else {
            ROS_ERROR("Arming service failed");
            return false;
        }
        if (!calibrateThrustModel(time)) {
            ROS_ERROR("Failed to calibrate thrust model");
            return false;
        }
        arm_time_=time;
        state_ = TakeoffState::PAUSE;
    }
    else if (state_ == TakeoffState::PAUSE){
        if (time > arm_time_ + ros::Duration(2.0)){
            state_ = TakeoffState::RAMP;
            if (!calibrateThrustModel(time)) {
                ROS_ERROR("Failed to calibrate thrust model");
                return false;
            }
            ramp_start_time_ = time;
        }
    }
    else if(state_ == TakeoffState::RAMP) {
        if (time <= ramp_start_time_ + ros::Duration(1.0)){
            double voltage;
            if (!battery_interpolator_.getInterpolatedMsgAtTime(voltage, time)) {
                ROS_ERROR("Failed to get battery voltage to calibrate thrust model");
                return false;
            }

            geometry_msgs::TransformStamped transform;
            bool success = transform_wrapper_.getTransformAtTime(transform,
                                                                 "map",
                                                                 "center_of_lift",
                                                                 time,
                                                                 update_timeout_);
            if (!success) {
                ROS_ERROR("Failed to get current transform to calibrate thrust model");
                return false;
            }

            geometry_msgs::PointStamped col_point;
            tf2::doTransform(col_point, col_point, transform);

            double hover_throttle = thrust_model_.throttleFromAccel(9.8, voltage, col_point.point.z);

            throttle_ = ((time-ramp_start_time_).toSec()/1.0) * hover_throttle;
            ROS_INFO("TARGET: %f", hover_throttle);
        }
        else{
            state_ = TakeoffState::DONE;
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
    ROS_INFO("THROTTLE: %f", throttle_);
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

    success = battery_interpolator_.waitUntilReady(startup_timeout_);
    if (!success) {
        ROS_ERROR("Failed to fetch battery voltage");
        return false;
    }

    const ros::Time start_time = ros::Time::now();
    while (ros::ok()
           && !landing_detected_message_received_
           && ros::Time::now() < start_time + startup_timeout_) {
        ros::spinOnce();
        ros::Duration(0.005).sleep();
    }

    if (!landing_detected_message_received_) {
        ROS_ERROR_STREAM("TakeoffController failed to fetch initial switch message");
        return false;
    }

    // This time is just used to calculate any ramping that needs to be done.
    last_update_time_ = std::max(landing_detected_message_.header.stamp,
                                 battery_interpolator_.getLastUpdateTime());
    return true;
}

bool TakeoffController::isDone()
{
  return (state_ == TakeoffState::DONE);
}

const ThrustModel& TakeoffController::getThrustModel() const
{
  return thrust_model_;
}

void TakeoffController::processLandingDetectedMessage(
    const iarc7_msgs::BoolStamped::ConstPtr& message)
{
    landing_detected_message_received_ = true;
    landing_detected_message_ = *message;
}

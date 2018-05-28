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
    : landing_detected_message_(),
      landing_detected_subscriber_(),
      landing_detected_message_received_(false),
      transform_wrapper_(),
      state_(LandState::DONE),
      requested_height_(0.0),
      actual_descend_rate_(0.0),
      descend_rate_(ros_utils::ParamUtils::getParam<double>(
              private_nh,
              "descend_rate")),
      descend_acceleration_(ros_utils::ParamUtils::getParam<double>(
              private_nh,
              "descend_acceleration")),
      last_update_time_(),
      startup_timeout_(ros_utils::ParamUtils::getParam<double>(
              private_nh,
              "startup_timeout")),
      update_timeout_(ros_utils::ParamUtils::getParam<double>(
              private_nh,
              "update_timeout")),
      uav_arm_client_(nh.serviceClient<iarc7_msgs::Arm>("uav_arm"))
{
    landing_detected_subscriber_ = nh.subscribe("landing_detected",
                                 100,
                                 &LandPlanner::processLandingDetectedMessage,
                                 this);
}

// Used to prepare and check initial conditions for landing
// getTargetTwist needs to begin being called shortly after this is called.
bool LandPlanner::prepareForTakeover(const ros::Time& time)
{
    if (time < last_update_time_) {
        ROS_ERROR("Tried to reset LandPlanner with time before last update");
        return false;
    }

    if(landing_detected_message_.data) {
        ROS_ERROR("Tried to reset the LandPlanner while being on the ground");
        return false;
    }

    // Get the current transform (xyz) of the quad
    geometry_msgs::TransformStamped transform;
    bool success = transform_wrapper_.getTransformAtTime(transform,
                                                         "map",
                                                         "level_quad",
                                                         time,
                                                         update_timeout_);
    if (!success) {
        ROS_ERROR("Failed to get current transform in LandPlanner::prepareForTakeover");
        return false;
    }

    requested_height_ = transform.transform.translation.z;

    actual_descend_rate_ = 0.0;

    state_ = LandState::DESCEND;
    // Mark the last update time as the current time since update may not have
    // Been called in a long time.
    last_update_time_ = time;
    return true;
}

// Main update
bool LandPlanner::getTargetMotionPoint(const ros::Time& time,
                         iarc7_msgs::MotionPointStamped& motion_point)
{
    if (time < last_update_time_) {
        ROS_ERROR("Tried to update LandPlanner with time before last update");
        return false;
    }

    // Get the current transform (xyz) of the quad
    geometry_msgs::TransformStamped transform;
    bool success = transform_wrapper_.getTransformAtTime(transform,
                                                         "map",
                                                         "level_quad",
                                                         time,
                                                         update_timeout_);
    if (!success) {
        ROS_ERROR("Failed to get current transform in LandPlanner::update");
        return false;
    }

    if(state_ == LandState::DESCEND)
    {
        actual_descend_rate_ = std::max(descend_rate_,
                                        actual_descend_rate_
                                        + (descend_acceleration_
                                        * (time - last_update_time_).toSec()));

        requested_height_ = std::max(0.0, requested_height_
                                          + (actual_descend_rate_
                                          * (time - last_update_time_).toSec()));

        if(landing_detected_message_.data) {
            // Sending disarm request to fc_comms
            iarc7_msgs::Arm srv;
            srv.request.data = false;
            // Check if request was succesful
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
    motion_point.header.stamp = time;

    motion_point.motion_point.pose.position.z = requested_height_;
    motion_point.motion_point.twist.linear.z = actual_descend_rate_;

    if(actual_descend_rate_ > descend_rate_) {
        motion_point.motion_point.accel.linear.z = descend_acceleration_;
    }

    last_update_time_ = time;
    return true;
}

bool LandPlanner::waitUntilReady()
{
    geometry_msgs::TransformStamped transform;
    bool success = transform_wrapper_.getTransformAtTime(transform,
                                                         "map",
                                                         "level_quad",
                                                         ros::Time(0),
                                                         startup_timeout_);
    if (!success)
    {
        ROS_ERROR("Failed to fetch transform");
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
        ROS_ERROR_STREAM("LandPlanner failed to fetch initial switch message");
        return false;
    }

    // This time is just used to calculate any ramping that needs to be done.
    last_update_time_ = landing_detected_message_.header.stamp;
    return true;
}

bool LandPlanner::isDone()
{
  return (state_ == LandState::DONE);
}

void LandPlanner::processLandingDetectedMessage(
    const iarc7_msgs::BoolStamped::ConstPtr& message)
{
    landing_detected_message_received_ = true;
    landing_detected_message_ = *message;
}

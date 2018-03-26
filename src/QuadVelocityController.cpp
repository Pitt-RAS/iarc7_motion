////////////////////////////////////////////////////////////////////////////
//
// Quad Velocity controller
//
// Accepts a target velocity and uses 4 PID loops (pitch, yaw, roll, thrust)
// To attempt to hit the target velocity.
//
////////////////////////////////////////////////////////////////////////////

#include <algorithm>
#include <cmath>
#include <boost/algorithm/clamp.hpp>

// Associated header
#include "iarc7_motion/QuadVelocityController.hpp"

// ROS Headers
#include "ros_utils/LinearMsgInterpolator.hpp"
#include "ros_utils/ParamUtils.hpp"
#include "ros_utils/SafeTransformWrapper.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

// ROS message headers
#include "geometry_msgs/AccelWithCovarianceStamped.h"
#include "iarc7_msgs/Float64Stamped.h"
#include "iarc7_msgs/MotionPointStamped.h"

using namespace Iarc7Motion;

QuadVelocityController::QuadVelocityController(
        double thrust_pid[6],
        double pitch_pid[6],
        double roll_pid[6],
        const ThrustModel& thrust_model,
        const ros::Duration& battery_timeout,
        ros::NodeHandle& nh,
        ros::NodeHandle& private_nh)
    : throttle_pid_(thrust_pid[0],
                    thrust_pid[1],
                    thrust_pid[2],
                    thrust_pid[3],
                    thrust_pid[4],
                    thrust_pid[5],
                    "throttle_pid",
                    private_nh),
      pitch_pid_(pitch_pid[0],
                 pitch_pid[1],
                 pitch_pid[2],
                 pitch_pid[3],
                 pitch_pid[4],
                 pitch_pid[5],
                 "pitch_pid",
                 private_nh),
      roll_pid_(roll_pid[0],
                roll_pid[1],
                roll_pid[2],
                roll_pid[3],
                roll_pid[4],
                roll_pid[5],
                "roll_pid",
                private_nh),
      thrust_model_(thrust_model),
      transform_wrapper_(),
      setpoint_(),
      startup_timeout_(ros_utils::ParamUtils::getParam<double>(
              private_nh,
              "startup_timeout")),
      update_timeout_(ros_utils::ParamUtils::getParam<double>(
              private_nh,
              "update_timeout")),
      accel_interpolator_(
              nh,
              "accel/filtered",
              update_timeout_,
              ros::Duration(0),
              [](const geometry_msgs::AccelWithCovarianceStamped& msg) {
                  return tf2::Vector3(msg.accel.accel.linear.x,
                                      msg.accel.accel.linear.y,
                                      msg.accel.accel.linear.z);
              },
              100),
      battery_interpolator_(nh,
                            "motor_battery",
                            update_timeout_,
                            battery_timeout,
                            [](const iarc7_msgs::Float64Stamped& msg) {
                                return msg.data;
                            },
                            100),
      odom_interpolator_(nh,
                         "odometry/filtered",
                         update_timeout_,
                         ros::Duration(0),
                         [](const nav_msgs::Odometry& msg) {
                             return tf2::Vector3(msg.twist.twist.linear.x,
                                                 msg.twist.twist.linear.y,
                                                 msg.twist.twist.linear.z);
                         },
                         100),
      min_thrust_(ros_utils::ParamUtils::getParam<double>(
              private_nh,
              "min_thrust")),
      max_thrust_(ros_utils::ParamUtils::getParam<double>(
              private_nh,
              "max_thrust")),
      level_flight_required_height_(
          ros_utils::ParamUtils::getParam<double>(
              private_nh,
              "level_flight_required_height")),
      level_flight_required_hysteresis_(
          ros_utils::ParamUtils::getParam<double>(
              private_nh,
              "level_flight_required_hysteresis")),
      level_flight_active_(true)
{
}

// Take in a target velocity that does not take into account the quads current heading
// And transform it to the velocity vectors that correspond to the quads current yaw
// Set the PID's set points accordingly
void QuadVelocityController::setTargetVelocity(geometry_msgs::Twist twist)
{
    setpoint_ = twist;
}

// Use a new thrust model
void QuadVelocityController::setThrustModel(const ThrustModel& thrust_model)
{
    thrust_model_ = thrust_model;
}

// Main update, runs all PID calculations and returns a desired uav_command
// Needs to be called at regular intervals in order to keep catching the latest velocities.
bool QuadVelocityController::update(const ros::Time& time,
                                    iarc7_msgs::OrientationThrottleStamped& uav_command,
                                    bool z_only,
                                    double pitch,
                                    double roll)
{
    if (time < last_update_time_) {
        ROS_ERROR("Tried to update QuadVelocityController with time before last update");
        return false;
    }

    // Get the current velocity of the quad.
    tf2::Vector3 velocity;
    bool success = odom_interpolator_.getInterpolatedMsgAtTime(velocity, time);
    if (!success) {
        ROS_ERROR("Failed to get current velocities in QuadVelocityController::update");
        return false;
    }

    // Get the current battery voltage of the quad
    double voltage;
    success = battery_interpolator_.getInterpolatedMsgAtTime(voltage, time);
    if (!success) {
        ROS_ERROR("Failed to get current battery voltage in QuadVelocityController::update");
        return false;
    }

    // Get the current acceleration of the quad
    tf2::Vector3 accel;
    success = accel_interpolator_.getInterpolatedMsgAtTime(accel, time);
    if (!success) {
        ROS_ERROR("Failed to get current acceleration in QuadVelocityController::update");
        return false;
    }

    // Get the current transform (rotation) of the quad
    geometry_msgs::TransformStamped transform;
    success = transform_wrapper_.getTransformAtTime(transform,
                                                    "level_quad",
                                                    "quad",
                                                    time,
                                                    update_timeout_);
    if (!success) {
        ROS_ERROR("Failed to get current transform in QuadVelocityController::update");
        return false;
    }

    // Get the current transform (rotation) of the quad
    geometry_msgs::TransformStamped col_height_transform;
    success = transform_wrapper_.getTransformAtTime(col_height_transform,
                                                    "map",
                                                    "center_of_lift",
                                                    time,
                                                    update_timeout_);
    if (!success) {
        ROS_ERROR("Failed to get current transform in QuadVelocityController::update");
        return false;
    }
    double col_height = col_height_transform.transform.translation.z;

    // Get current yaw from the transform
    double current_yaw = yawFromQuaternion(transform.transform.rotation);

    // Update setpoints on PID controllers
    updatePidSetpoints(current_yaw);

    // Update all the PID loops

    // Used to temporarily store throttle and angle outputs from PID loops
    double vertical_accel_output;
    double pitch_output;
    double roll_output;

    // Update throttle PID loop
    success = throttle_pid_.update(velocity.z(),
                                   time,
                                   vertical_accel_output,
                                   accel.z(), true);

    if (!success) {
        ROS_ERROR("Throttle PID update failed in QuadVelocityController::update");
        return false;
    }

    // Calculate local frame velocities
    double local_x_velocity = std::cos(current_yaw) * velocity.x()
                            + std::sin(current_yaw) * velocity.y();
    double local_y_velocity = -std::sin(current_yaw) * velocity.x()
                            +  std::cos(current_yaw) * velocity.y();
    // Calculate local frame accelerations
    double local_x_accel = std::cos(current_yaw) * accel.x()
                         + std::sin(current_yaw) * accel.y();
    double local_y_accel = -std::sin(current_yaw) * accel.x()
                         +  std::cos(current_yaw) * accel.y();

    if (level_flight_active_ && col_height 
                                  > level_flight_required_height_
                                    + level_flight_required_hysteresis_) {
        level_flight_active_ = false;
        pitch_pid_.resetAccumulator();
        roll_pid_.resetAccumulator();
    }

    if (z_only) {
        pitch_output = pitch;
        roll_output = roll;
    }
    else if (col_height < level_flight_required_height_ || level_flight_active_) {
        pitch_output = 0.0f;
        roll_output = 0.0f;
        level_flight_active_ = true;
    }
    else {
        // Update pitch PID loop
        success = pitch_pid_.update(local_x_velocity,
                                    time,
                                    pitch_output,
                                    local_x_accel);
        if (!success) {
            ROS_ERROR("Pitch PID update failed in QuadVelocityController::update");
            return false;
        }

        // Update roll PID loop
        success = roll_pid_.update(-local_y_velocity,
                                   time,
                                   roll_output,
                                   -local_y_accel);
        if (!success) {
            ROS_ERROR("Roll PID update failed in QuadVelocityController::update");
            return false;
        }
    }

    // Fill in the uav_command's information
    uav_command.header.stamp = time;

    double hover_accel = 9.8;
    //based on roll and pitch angles we calculate additional throttle to match the level hover_accel
    double tilt_accel = hover_accel*(1-cos(roll_output)*cos(pitch_output));


    // Simple feedforward using a fixed hover_accel to avoid excessive
    // oscillations from the PID's I term compensating for there needing to be
    // an average throttle value at 0 velocity in the z axis.
    ROS_DEBUG("Thrust: %f, Voltage: %f, height: %f", hover_accel + tilt_accel + vertical_accel_output, voltage, col_height);
    ROS_ERROR("Hover: %f, Tilt: %f, Vertical %f", hover_accel, tilt_accel, vertical_accel_output);
    double thrust_request = hover_accel + tilt_accel + vertical_accel_output;
    uav_command.throttle = thrust_model_.voltageFromThrust(
            std::min(std::max(thrust_request, min_thrust_), max_thrust_),
            col_height)
            / voltage;
    uav_command.data.pitch = pitch_output;
    uav_command.data.roll = roll_output;

    // Yaw rate needs no correction because the input and output are both
    // velocities
    uav_command.data.yaw = -setpoint_.angular.z;

    // Check that the PID loops did not return invalid values before returning
    if (!std::isfinite(uav_command.throttle)
     || !std::isfinite(uav_command.data.pitch)
     || !std::isfinite(uav_command.data.roll)
     || !std::isfinite(uav_command.data.yaw)) {
        ROS_ERROR(
            "Part of command is not finite in QuadVelocityController::update");
        return false;
    }

    // Print the velocity and throttle information
    ROS_DEBUG("Vz: %f Vx: %f Vy: %f",
             velocity.z(),
             velocity.x(),
             velocity.y());
    ROS_DEBUG("Throttle: %f Pitch: %f Roll: %f Yaw: %f",
             uav_command.throttle,
             uav_command.data.pitch,
             uav_command.data.roll,
             uav_command.data.yaw);

    last_update_time_ = time;
    return true;
}

bool QuadVelocityController::waitUntilReady()
{
    bool success = accel_interpolator_.waitUntilReady(startup_timeout_);
    if (!success) {
        ROS_ERROR("Failed to fetch initial acceleration");
        return false;
    }

    success = battery_interpolator_.waitUntilReady(startup_timeout_);
    if (!success) {
        ROS_ERROR("Failed to fetch battery voltage");
        return false;
    }

    success = odom_interpolator_.waitUntilReady(startup_timeout_);
    if (!success) {
        ROS_ERROR("Failed to fetch initial velocity");
        return false;
    }

    geometry_msgs::TransformStamped transform;
    success = transform_wrapper_.getTransformAtTime(transform,
                                                    "level_quad",
                                                    "quad",
                                                    ros::Time(0),
                                                    startup_timeout_);
    if (!success)
    {
        ROS_ERROR("Failed to fetch initial transform level_quad to quad");
        return false;
    }

    success = transform_wrapper_.getTransformAtTime(transform,
                                                    "map",
                                                    "center_of_lift",
                                                    ros::Time(0),
                                                    startup_timeout_);
    if (!success)
    {
        ROS_ERROR("Failed to fetch initial transform map to center_of_lift");
        return false;
    }

    // Mark the last update time as 1ns later than the odometry message,
    // because we should always have a message older than the last update
    last_update_time_ = std::max({accel_interpolator_.getLastUpdateTime(),
                                  battery_interpolator_.getLastUpdateTime(),
                                  odom_interpolator_.getLastUpdateTime()});
    return true;
}

void QuadVelocityController::updatePidSetpoints(double current_yaw)
{
    throttle_pid_.setSetpoint(setpoint_.linear.z);

    // Pitch and roll velocities are transformed according to the last yaw
    // angle because the incoming target velocities are in the map frame
    double local_x_velocity = setpoint_.linear.x * std::cos(current_yaw)
                            + setpoint_.linear.y * std::sin(current_yaw);
    double local_y_velocity = setpoint_.linear.x * -std::sin(current_yaw)
                            + setpoint_.linear.y *  std::cos(current_yaw);

    pitch_pid_.setSetpoint(local_x_velocity);

    // Note: Roll is inverted because a positive y velocity means a negative
    // roll by the right hand rule
    roll_pid_.setSetpoint(-local_y_velocity);
}

double QuadVelocityController::yawFromQuaternion(
        const geometry_msgs::Quaternion& rotation)
{
    tf2::Quaternion quaternion;
    tf2::convert(rotation, quaternion);

    tf2::Matrix3x3 matrix;
    matrix.setRotation(quaternion);

    double y, p, r;
    matrix.getEulerYPR(y, p, r);

    return y;
}

bool QuadVelocityController::prepareForTakeover()
{
    throttle_pid_.resetAccumulator();
    pitch_pid_.resetAccumulator();
    roll_pid_.resetAccumulator();
    return true;
}

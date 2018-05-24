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
        double vz_pid_settings[6],
        double vx_pid_settings[6],
        double vy_pid_settings[6],
        double& height_p,
        const ThrustModel& thrust_model,
        const ThrustModel& thrust_model_side,
        const ros::Duration& battery_timeout,
        ros::NodeHandle& nh,
        ros::NodeHandle& private_nh)
    : vz_pid_(vz_pid_settings,
              "vz_pid",
              private_nh),
      vx_pid_(vx_pid_settings,
              "vx_pid",
              private_nh),
      vy_pid_(vy_pid_settings,
              "vy_pid",
              private_nh),
      thrust_model_(thrust_model),
      thrust_model_front_(thrust_model_side),
      thrust_model_back_(thrust_model_side),
      thrust_model_left_(thrust_model_side),
      thrust_model_right_(thrust_model_side),
      transform_wrapper_(),
      setpoint_(),
      xy_mixer_(ros_utils::ParamUtils::getParam<std::string>(
              private_nh,
              "xy_mixer")),
      height_p_(height_p),
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
                              Eigen::VectorXd v(6);
                              v[0] = msg.twist.twist.linear.x;
                              v[1] = msg.twist.twist.linear.y;
                              v[2] = msg.twist.twist.linear.z;
                              v[3] = msg.pose.pose.position.x;
                              v[4] = msg.pose.pose.position.y;
                              v[5] = msg.pose.pose.position.z;
                              return v;
                         },
                         100),
      min_thrust_(ros_utils::ParamUtils::getParam<double>(
              private_nh,
              "min_thrust")),
      max_thrust_(ros_utils::ParamUtils::getParam<double>(
              private_nh,
              "max_thrust")),
      min_side_thrust_(ros_utils::ParamUtils::getParam<double>(
              private_nh,
              "min_side_thrust")),
      max_side_thrust_(ros_utils::ParamUtils::getParam<double>(
              private_nh,
              "max_side_thrust")),
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
void QuadVelocityController::setTargetVelocity(iarc7_msgs::MotionPointStamped motion_point)
{
    setpoint_ = motion_point;
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
                                    bool xy_passthrough_mode,
                                    double a_x,
                                    double a_y)
{
    if (time < last_update_time_) {
        ROS_ERROR("Tried to update QuadVelocityController with time before last update");
        return false;
    }

    // Get the current odometry of the quad.
    Eigen::VectorXd odometry;
    bool success = odom_interpolator_.getInterpolatedMsgAtTime(odometry, time);
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
    updatePidSetpoints(current_yaw, odometry);

    // Update Vz PID loop with position and velocity
    double x_accel_output = 0;
    double y_accel_output = 0;
    double z_accel_output = 0;
    success = vz_pid_.update(odometry[2],
                             time,
                             z_accel_output,
                             accel.z() - setpoint_.motion_point.accel.linear.z, true);

    if (!success) {
        ROS_ERROR("Vz PID update failed in QuadVelocityController::update");
        return false;
    }

    // Calculate local frame velocities
    double local_x_velocity = std::cos(current_yaw) * odometry[0]
                            + std::sin(current_yaw) * odometry[1];
    double local_y_velocity = -std::sin(current_yaw) * odometry[0]
                            +  std::cos(current_yaw) * odometry[1];
    // Calculate local frame accelerations
    double local_x_accel = std::cos(current_yaw) * accel.x()
                         + std::sin(current_yaw) * accel.y();
    double local_y_accel = -std::sin(current_yaw) * accel.x()
                         +  std::cos(current_yaw) * accel.y();

    const auto& setpoint_accel = setpoint_.motion_point.accel.linear;
    double local_x_setpoint_accel = std::cos(current_yaw) * setpoint_accel.x
                                  + std::sin(current_yaw) * setpoint_accel.y;
    double local_y_setpoint_accel = -std::sin(current_yaw) * setpoint_accel.x
                                  +  std::cos(current_yaw) * setpoint_accel.y;

    if (level_flight_active_ && col_height 
                                  > level_flight_required_height_
                                    + level_flight_required_hysteresis_) {
        level_flight_active_ = false;
        vx_pid_.resetAccumulator();
        vy_pid_.resetAccumulator();
    }

    // Final output variables
    if (xy_passthrough_mode) {
        x_accel_output = a_x;
        y_accel_output = a_y;
    }
    else if (col_height < level_flight_required_height_ || level_flight_active_) {
        x_accel_output = 0;
        y_accel_output = 0;
        level_flight_active_ = true;
    }
    else {
        // Update vx PID loop
        success = vx_pid_.update(local_x_velocity,
                                 time,
                                 x_accel_output,
                                 local_x_accel - local_x_setpoint_accel);
        if (!success) {
            ROS_ERROR("Vx PID update failed in QuadVelocityController::update");
            return false;
        }

        // Update vy PID loop
        success = vy_pid_.update(local_y_velocity,
                                 time,
                                 y_accel_output,
                                 local_y_accel - local_y_setpoint_accel);
        if (!success) {
            ROS_ERROR("Vy PID update failed in QuadVelocityController::update");
            return false;
        }
    }

    // Fill in the uav_command's information
    uav_command.header.stamp = time;

    // TODO add accel setpoints from plan here
    double x_accel = x_accel_output + local_x_setpoint_accel;
    double y_accel = y_accel_output + local_y_setpoint_accel;
    double z_accel = g_ + z_accel_output + setpoint_accel.z;

    double thrust_request;
    if(xy_mixer_ == "4dof") {
        double pitch_request, roll_request;
        {
            Eigen::Vector3d accel;
            accel(0) = x_accel;
            accel(1) = y_accel;
            accel(2) = z_accel;
            commandForAccel(accel, pitch_request, roll_request, thrust_request);
        }

        uav_command.data.pitch = pitch_request;
        uav_command.data.roll = roll_request;

        uav_command.planar.front_throttle = 0;
        uav_command.planar.back_throttle = 0;
        uav_command.planar.left_throttle = 0;
        uav_command.planar.right_throttle = 0;
    }
    else if(xy_mixer_ == "6dof") {
        //ROS_ERROR_STREAM(x_accel << " " << y_accel);
        //ROS_ERROR_STREAM(min_side_thrust_ << " " << max_side_thrust_);

        thrust_request = z_accel;

        uav_command.data.pitch = 0;
        uav_command.data.roll = 0;

        uav_command.planar.front_throttle = thrust_model_front_.voltageFromThrust(
            std::min(std::max(-x_accel + min_side_thrust_, min_side_thrust_), max_side_thrust_),
            1,
            0.0)
            / voltage;
        uav_command.planar.back_throttle = thrust_model_back_.voltageFromThrust(
          std::min(std::max(x_accel + min_side_thrust_, min_side_thrust_), max_side_thrust_),
            1,
            0.0)
            / voltage;
        uav_command.planar.left_throttle = thrust_model_left_.voltageFromThrust(
            std::min(std::max(-y_accel + min_side_thrust_, min_side_thrust_), max_side_thrust_),
            1,
            0.0)
            / voltage;
        uav_command.planar.right_throttle = thrust_model_right_.voltageFromThrust(
            std::min(std::max(y_accel + min_side_thrust_, min_side_thrust_), max_side_thrust_),
            1,
            0.0)
            / voltage;
        //ROS_ERROR_STREAM(uav_command);
    }
    else {
      ROS_ERROR("Invalid XY Mixer type");
      return false;
    }

    ROS_DEBUG("Thrust: %f, Voltage: %f, height: %f", thrust_request, voltage, col_height);
    uav_command.throttle = thrust_model_.voltageFromThrust(
            std::min(std::max(thrust_request, min_thrust_), max_thrust_),
            4,
            col_height)
            / voltage;

    // Yaw rate needs no correction because the input and output are both
    // velocities
    uav_command.data.yaw = -setpoint_.motion_point.twist.angular.z;

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
             odometry[2],
             odometry[0],
             odometry[1]);
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

void QuadVelocityController::updatePidSetpoints(double current_yaw, Eigen::VectorXd& odometry)
{
    double position_velocity_request = height_p_ * 
      (setpoint_.motion_point.pose.position.z - odometry[5]);

    double velocity_request = position_velocity_request + setpoint_.motion_point.twist.linear.z;

    vz_pid_.setSetpoint(velocity_request);

    // x and y velocities are transformed according to the last yaw
    // angle because the incoming target velocities are in the map frame
    double local_x_velocity = setpoint_.motion_point.twist.linear.x * std::cos(current_yaw)
                            + setpoint_.motion_point.twist.linear.y * std::sin(current_yaw);
    double local_y_velocity = setpoint_.motion_point.twist.linear.x * -std::sin(current_yaw)
                            + setpoint_.motion_point.twist.linear.y *  std::cos(current_yaw);

    vx_pid_.setSetpoint(local_x_velocity);
    vy_pid_.setSetpoint(local_y_velocity);
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

void QuadVelocityController::commandForAccel(
        const Eigen::Vector3d& accel,
        double& pitch,
        double& roll,
        double& thrust)
{
    const auto a_hat = accel.normalized();
    roll = -std::asin(a_hat(1));
    const auto a_no_roll = a_hat - a_hat(1) * Eigen::Vector3d::UnitY();
    const auto a_hat_no_roll = a_no_roll.normalized();
    pitch = std::asin(a_hat_no_roll(0));
    thrust = accel.norm();
}

bool QuadVelocityController::prepareForTakeover()
{
    vz_pid_.reset();
    vx_pid_.reset();
    vy_pid_.reset();
    return true;
}

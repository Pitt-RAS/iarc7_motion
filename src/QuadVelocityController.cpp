////////////////////////////////////////////////////////////////////////////
//
// Quad Velocity controller
//
// Accepts a target velocity and uses 4 PID loops (pitch, yaw, roll, thrust)
// To attempt to hit the target velocity.
//
////////////////////////////////////////////////////////////////////////////

#include <cmath>

// Associated header
#include "iarc7_motion/QuadVelocityController.hpp"

// ROS Headers
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

// ROS message headers

using namespace Iarc7Motion;

template<typename T>
static T getParam(ros::NodeHandle& nh, const std::string& name)
{
    T val;
    ROS_ASSERT_MSG(nh.getParam(name, val),
                   "Failed to retrieve parameter: %s",
                   name.c_str());
    return val;
}

// Create QuadVelocityController, copy PID settings, initialize all other variables
QuadVelocityController::QuadVelocityController(double thrust_pid[5],
                                               double pitch_pid[5],
                                               double roll_pid[5],
                                               double hover_throttle,
                                               ros::NodeHandle& nh,
                                               ros::NodeHandle& private_nh) :
throttle_pid_(thrust_pid[0], thrust_pid[1], thrust_pid[2], thrust_pid[3], thrust_pid[4]),
pitch_pid_(pitch_pid[0], pitch_pid[1], pitch_pid[2], pitch_pid[3], pitch_pid[4]),
roll_pid_(roll_pid[0], roll_pid[1], roll_pid[2], roll_pid[3], roll_pid[4]),
tfBuffer_(),
tfListener_(tfBuffer_),
odometry_subscriber_(nh.subscribe(
                         "odometry/filtered",
                         100,
                         &QuadVelocityController::odometryCallback,
                         this)),
odometry_msg_queue_(),
setpoint_(),
hover_throttle_(hover_throttle),
startup_timeout_(getParam<double>(private_nh, "startup_timeout")),
update_timeout_(getParam<double>(private_nh, "update_timeout"))
{
}

// Take in a target velocity that does not take into account the quads current heading
// And transform it to the velocity vectors that correspond to the quads current yaw
// Set the PID's set points accordingly
void QuadVelocityController::setTargetVelocity(geometry_msgs::Twist twist)
{
    setpoint_ = twist;
}

// Main update, runs all PID calculations and returns a desired uav_command
// Needs to be called at regular intervals in order to keep catching the latest velocities.
bool QuadVelocityController::update(const ros::Time& time,
                                    iarc7_msgs::OrientationThrottleStamped& uav_command)
{
    if (time < last_update_time_) {
        ROS_ERROR("Tried to update QuadVelocityController with time before last update");
        return false;
    }

    // Get the current velocity of the quad.
    geometry_msgs::Vector3 velocity;
    bool success = getVelocityAtTime(velocity, time, update_timeout_);
    if (!success) {
        ROS_WARN("Failed to get current velocities in QuadVelocityController::update");
        return false;
    }

    // Get the current transform (rotation) of the quad
    geometry_msgs::TransformStamped transform;
    success = getTransformAtTime(transform, time, update_timeout_);
    if (!success) {
        ROS_WARN("Failed to get current transform in QuadVelocityController::update");
        return false;
    }

    // Get current yaw from the transform
    double current_yaw = yawFromQuaternion(transform.transform.rotation);

    // Update setpoints on PID controllers
    updatePidSetpoints(current_yaw);

    // Update all the PID loops

    //hover throttle adjustment for tilting
    double tilt_throttle;
    // Used to temporarily store throttle and angle outputs from PID loops
    double throttle_output;
    double pitch_output;
    double roll_output;

    // Update throttle PID loop
    success = throttle_pid_.update(velocity.z, time, throttle_output);
    if (!success) {
        ROS_WARN("Throttle PID update failed in QuadVelocityController::update");
        return false;
    }

    // Calculate local frame velocities
    double local_x_velocity = std::cos(current_yaw) * velocity.x
                            - std::sin(current_yaw) * velocity.y;
    double local_y_velocity = std::sin(current_yaw) * velocity.x
                            - std::cos(current_yaw) * velocity.y;

    // Update pitch PID loop
    success = pitch_pid_.update(local_x_velocity, time, pitch_output);
    if (!success) {
        ROS_WARN("Pitch PID update failed in QuadVelocityController::update");
        return false;
    }

    // Update roll PID loop
    success = roll_pid_.update(local_y_velocity, time, roll_output);
    if (!success) {
        ROS_WARN("Roll PID update failed in QuadVelocityController::update");
        return false;
    }

    // Fill in the uav_command's information
    uav_command.header.stamp = time;

    //based on roll and pitch angles we calculate additional throttle to match the level hover_throttle_
    tilt_throttle = hover_throttle_*(1-cos(roll_output)*cos(pitch_output));

    // Simple feedforward using a fixed hover_throttle_ to avoid excessive
    // oscillations from the PID's I term compensating for there needing to be
    // an average throttle value at 0 velocity in the z axis.
    uav_command.throttle = throttle_output + hover_throttle_ + tilt_throttle;
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
        ROS_WARN(
            "Part of command is not finite in QuadVelocityController::update");
        return false;
    }

    // Print the velocity and throttle information
    ROS_INFO("Vz: %f Vx: %f Vy: %f",
             velocity.z,
             velocity.x,
             velocity.y);
    ROS_INFO("Throttle: %f Pitch: %f Roll: %f Yaw: %f",
             uav_command.throttle,
             uav_command.data.pitch,
             uav_command.data.roll,
             uav_command.data.yaw);

    last_update_time_ = time;
    return true;
}

bool QuadVelocityController::waitUntilReady()
{
    const ros::Time start_time = ros::Time::now();
    while (ros::ok()
           && odometry_msg_queue_.empty()
           && ros::Time::now() < start_time + startup_timeout_) {
        ros::spinOnce();
        ros::Duration(0.005).sleep();
    }

    if (odometry_msg_queue_.empty()) {
        ROS_ERROR("Failed to fetch initial velocity");
        return false;
    }

    geometry_msgs::TransformStamped transform;
    bool success = getTransformAtTime(transform,
                                      ros::Time(0),
                                      startup_timeout_);

    if (!success)
    {
        ROS_ERROR("Failed to fetch initial transform");
        return false;
    }
    else
    {
        // Mark the last update time as 1ns later than the odometry message,
        // because we should always have a message older than the last update
        last_update_time_ = odometry_msg_queue_.front().header.stamp
                          + ros::Duration(0, 1);
        return true;
    }
}

// Get a valid transform, blocks until the transform is received or
// the request takes too long and times out
bool QuadVelocityController::getTransformAtTime(
        geometry_msgs::TransformStamped& transform,
        const ros::Time& time,
        const ros::Duration& timeout) const
{
    const ros::Time start_time = ros::Time::now();

    // Catch TransformException exceptions
    try
    {
        // While we aren't supposed to be shutting down
        while (ros::ok())
        {
            if (ros::Time::now() > start_time + timeout)
            {
                ROS_ERROR_STREAM("Transform timed out ("
                              << "current time: " << ros::Time::now() << ", "
                              << "request time: " << start_time
                              << ")");
                return false;
            }

            // Check if the transform from map to quad can be made right now
            if (tfBuffer_.canTransform("level_quad", "quad", time))
            {
                // Get the transform
                transform = tfBuffer_.lookupTransform("level_quad", "quad", time);
                return true;
            }

            // Handle callbacks and sleep for a small amount of time
            // before looping again
            ros::spinOnce();
            ros::Duration(0.005).sleep();
        }
    }
    // Catch any exceptions that might happen while transforming
    catch (tf2::TransformException& ex)
    {
        ROS_ERROR("Exception transforming level_quad to quad: %s", ex.what());
    }

    ROS_ERROR("Exception or ros::ok was false");
    return false;
}

bool QuadVelocityController::getVelocityAtTime(
        geometry_msgs::Vector3& velocity,
        const ros::Time& time,
        const ros::Duration& timeout)
{
    // Wait until there's a message with stamp >= time
    if (!waitForOdometryAtTime(time, timeout))
    {
        ROS_ERROR("Timed out waiting for odometry message in getVelocityAtTime");
        return false;
    }

    // Get first msg that is >= the requested time
    std::vector<nav_msgs::Odometry>::iterator geq_msg
        = std::lower_bound(odometry_msg_queue_.begin(),
                           odometry_msg_queue_.end(),
                           time,
                           &QuadVelocityController::timeVsMsgStampedComparator
                                <nav_msgs::Odometry>);

    ROS_ERROR_COND(geq_msg <= odometry_msg_queue_.begin(),
                   "geq_msg - 1 out of bounds");
    ROS_ERROR_COND(geq_msg >= odometry_msg_queue_.end(),
                   "geq_msg out of bounds");

    // Linear interpolation between next_odom and last_odom
    const nav_msgs::Odometry& next_odom = *geq_msg;
    const nav_msgs::Odometry& last_odom = *(geq_msg - 1);

    const geometry_msgs::Vector3& next_vel = next_odom.twist.twist.linear;
    const geometry_msgs::Vector3& last_vel = last_odom.twist.twist.linear;

    double dt = (next_odom.header.stamp - last_odom.header.stamp).toSec();

    velocity.x = ((time - last_odom.header.stamp).toSec() / dt) * next_vel.x
               + ((next_odom.header.stamp - time).toSec() / dt) * last_vel.x;
    velocity.y = ((time - last_odom.header.stamp).toSec() / dt) * next_vel.y
               + ((next_odom.header.stamp - time).toSec() / dt) * last_vel.y;
    velocity.z = ((time - last_odom.header.stamp).toSec() / dt) * next_vel.z
               + ((next_odom.header.stamp - time).toSec() / dt) * last_vel.z;
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

bool QuadVelocityController::waitForOdometryAtTime(
        const ros::Time& time,
        const ros::Duration& timeout)
{
    if (odometry_msg_queue_.front().header.stamp >= time) {
        ROS_ERROR("Class invariant false: odometry_msg_queue_ does not contain a message older than the requested time");
        return false;
    }

    const ros::Time start_time = ros::Time::now();
    while (ros::ok()
           && odometry_msg_queue_.back().header.stamp < time
           && ros::Time::now() < start_time + timeout) {
        ros::spinOnce();
        ros::Duration(0.005).sleep();
    }

    return (odometry_msg_queue_.back().header.stamp >= time);
}

void QuadVelocityController::odometryCallback(const nav_msgs::Odometry& msg)
{
    if (!odometry_msg_queue_.empty()
          && msg.header.stamp < odometry_msg_queue_.back().header.stamp) {
        ROS_ERROR_STREAM("Ignoring odometry message with timestamp of "
                      << msg.header.stamp
                      << " because the queue already has a message with stamp "
                      << odometry_msg_queue_.back().header.stamp);
        return;
    }

    odometry_msg_queue_.push_back(msg);

    // Keep only one message older than the last update time
    std::vector<nav_msgs::Odometry>::const_iterator first_geq_time
        = std::lower_bound(odometry_msg_queue_.begin(),
                           odometry_msg_queue_.end(),
                           last_update_time_,
                           &QuadVelocityController::timeVsMsgStampedComparator
                                <nav_msgs::Odometry>);
    if (first_geq_time - 1 > odometry_msg_queue_.begin()) {
        odometry_msg_queue_.erase(odometry_msg_queue_.begin(),
                                  first_geq_time - 1);
    }
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

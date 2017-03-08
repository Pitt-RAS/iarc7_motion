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

// Create QuadVelocityController, copy PID settings, initialize all other variables
QuadVelocityController::QuadVelocityController(double thrust_pid[5],
                                               double pitch_pid[5],
                                               double roll_pid[5],
                                               double yaw_pid[5],
                                               double hover_throttle,
                                               double expected_update_frequency) :
tfBuffer_(),
tfListener_(tfBuffer_),
throttle_pid_(thrust_pid[0], thrust_pid[1], thrust_pid[2], thrust_pid[3], thrust_pid[4]),
pitch_pid_(pitch_pid[0], pitch_pid[1], pitch_pid[2], pitch_pid[3], pitch_pid[4]),
roll_pid_(roll_pid[0], roll_pid[1], roll_pid[2], roll_pid[3], roll_pid[4]),
yaw_pid_(yaw_pid[0], yaw_pid[1], yaw_pid[2], yaw_pid[3], yaw_pid[4]),
last_transform_stamped_(),
have_last_transform_(false),
hover_throttle_(hover_throttle),
expected_update_frequency_(expected_update_frequency)
{

}

// Take in a target velocity that does not take into account the quads current heading
// And transform it to the velocity vectors that correspond to the quads current yaw
// Set the PID's set points accordingly
void QuadVelocityController::setTargetVelocity(geometry_msgs::Twist twist)
{
    // Throttle and yaw_pid need no correction.
    // Remember these are velocities.
    throttle_pid_.setSetpoint(twist.linear.z);
    yaw_pid_.setSetpoint(twist.angular.z);

    double last_yaw
        = yawFromQuaternion(last_transform_stamped_.transform.rotation);

    // Pitch and roll velocities are transformed according to the last_yaw
    // angle because the incoming target velocities do not take the current yaw
    // into account
    pitch_pid_.setSetpoint(twist.linear.x * std::cos(last_yaw)
                         + twist.linear.y * std::sin(last_yaw));

    // Note: Roll is inverted because a positive y velocity means a negative
    // roll by the right hand rule
    roll_pid_.setSetpoint(-(twist.linear.x * -std::sin(last_yaw)
                          + twist.linear.y * std::cos(last_yaw)));
}

// Main update, runs all PID calculations and returns a desired uav_command
// Needs to be called at regular intervals in order to keep catching the latest velocities.
bool QuadVelocityController::update(const ros::Time& time,
                                    iarc7_msgs::OrientationThrottleStamped& uav_command)
{
    // Get the current velocity of the quad.
    geometry_msgs::Twist velocity;
    bool success = getVelocityAtTime(velocity, time);
    if (!success) {
        ROS_WARN("Failed to get current velocities in QuadVelocityController::update");
        return false;
    }

    // Update all the PID loops

    //hover throttle adjustment for tilting
    double tilt_throttle;
    // Used to temporarily store throttle and angle outputs from PID loops
    double throttle_output;
    double pitch_output;
    double roll_output;
    double yaw_output;

    // Update throttle PID loop
    success = throttle_pid_.update(velocity.linear.z, time, throttle_output);
    if (!success) {
        ROS_WARN("Throttle PID update failed in QuadVelocityController::update");
        return false;
    }

    // Update pitch PID loop
    success = pitch_pid_.update(velocity.linear.x, time, pitch_output);
    if (!success) {
        ROS_WARN("Pitch PID update failed in QuadVelocityController::update");
        return false;
    }

    // Update roll PID loop
    success = roll_pid_.update(-velocity.linear.y, time, roll_output);
    if (!success) {
        ROS_WARN("Roll PID update failed in QuadVelocityController::update");
        return false;
    }

    // Update yaw PID loop
    success = yaw_pid_.update(velocity.angular.z, time, yaw_output);
    if (!success) {
        ROS_WARN("Yaw PID update failed in QuadVelocityController::update");
        return false;
    }

    // Fill in the uav_command's information
    uav_command.header.stamp = time;

    //based on roll and pitch angles we calculate additional throttle to match the level hover_throttle_
    tilt_throttle = hover_throttle_*(1-cos(roll_output)*cos(pitch_output));

    // Simple feedforward using a fixed hover_throttle_ to avoid excessive oscillations from the
    // PID's I term compensating for there needing to be an  average throttle value at 0 velocity in the z axis.
    uav_command.throttle = throttle_output + hover_throttle_ + tilt_throttle;
    uav_command.data.pitch = pitch_output;
    uav_command.data.roll = roll_output;
    uav_command.data.yaw = yaw_output;

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
    ROS_INFO("Vz: %f Vx: %f Vy: %f Vyaw: %f",
             velocity.linear.z,
             velocity.linear.x,
             velocity.linear.y,
             velocity.angular.z);
    ROS_INFO("Throttle: %f Pitch: %f Roll: %f Yaw: %f",
             uav_command.throttle,
             uav_command.data.pitch,
             uav_command.data.roll,
             uav_command.data.yaw);

    return true;
}

bool QuadVelocityController::waitUntilReady()
{
    geometry_msgs::TransformStamped transform;
    bool success = getTransformAtTime(
                       transform,
                       ros::Time(0),
                       ros::Duration(INITIAL_TRANSFORM_WAIT_SECONDS));
    if (!success)
    {
        ROS_ERROR("Failed to fetch initial transform");
    }

    return success;
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
            if (tfBuffer_.canTransform("map", "quad", time))
            {
                // Get the transform
                transform = tfBuffer_.lookupTransform("map", "quad", time);
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
        ROS_ERROR("Exception transforming map to level_quad: %s", ex.what());
    }

    ROS_ERROR("Exception or ros::ok was false");
    return false;
}

// Gets the velocity from the transformation information
// Blocks while waiting for a new transform to calculate the velocity with
bool QuadVelocityController::getVelocityAtTime(
        geometry_msgs::Twist& return_velocities,
        const ros::Time& time)
{
    const ros::Duration timeout = ros::Duration(MAX_TRANSFORM_WAIT_SECONDS);

    // Check if the quad velocity controller has not been run once
    if (!have_last_transform_)
    {
        ros::Time last_time = time
                            - ros::Duration(1.0 / expected_update_frequency_);

        // Get a brand new transform
        // Blocks until able to transform at the requested time
        bool fetched_transform = getTransformAtTime(last_transform_stamped_,
                                                    last_time,
                                                    timeout);

        // Check if the call failed
        if (!fetched_transform)
        {
            ROS_ERROR("Failed to fetch first transform in QuadVelocityController");
            have_last_transform_ = false;
            return false;
        }

        have_last_transform_ = true;
    }

    // Get a brand new transform
    // Blocks until able to transform at the requested time
    geometry_msgs::TransformStamped transform_stamped;
    bool fetched_transform = getTransformAtTime(transform_stamped,
                                                time,
                                                timeout);

    // Check if the call failed
    if (!fetched_transform)
    {
        ROS_ERROR("Failed to fetch new transform in QuadVelocityController");
        have_last_transform_ = false;
        return false;
    }

    // Make sure the difference between transforms isn't too much
    // Calculate the lastest time between transforms that is allowed.
    ros::Time latest_time_allowed = last_transform_stamped_.header.stamp
                + ros::Duration(MAX_TRANSFORM_DIFFERENCE_SECONDS);
    // Make the sure the transform didn't come after the last time
    // that was acceptable.
    if (transform_stamped.header.stamp > latest_time_allowed)
    {
        ROS_ERROR_STREAM(
            "First available transform came after latest allowed time ("
         << "transform time: " << transform_stamped.header.stamp << ", "
         << "latest allowed time: " << latest_time_allowed
         << ")");
        return false;
    }

    return_velocities = twistFromTransforms(last_transform_stamped_,
                                            transform_stamped);

    // Store the old transform
    last_transform_stamped_ = transform_stamped;

    return true;
}

geometry_msgs::Twist QuadVelocityController::twistFromTransforms(
        const geometry_msgs::TransformStamped& transform1,
        const geometry_msgs::TransformStamped& transform2)
{
    geometry_msgs::Twist result;

    // Get the time between the two transforms
    ros::Duration dt = transform2.header.stamp - transform1.header.stamp;
    double delta_seconds = dt.toSec();

    // Calculate world_x and world_y velocities
    double world_x_vel = (transform2.transform.translation.x
                        - transform1.transform.translation.x) / delta_seconds;
    double world_y_vel = (transform2.transform.translation.y
                        - transform1.transform.translation.y) / delta_seconds;

    double current_yaw = yawFromQuaternion(transform2.transform.rotation);

    // Transform the world_x and world_y velocities according to the quads
    // current heading
    result.linear.x = world_x_vel * std::cos(current_yaw)
                    + world_y_vel * std::sin(current_yaw);
    result.linear.y = world_x_vel * -std::sin(current_yaw)
                    + world_y_vel * std::cos(current_yaw);

    // Calculate the world_z velocity, no transform is needed
    result.linear.z = (transform2.transform.translation.z
                     - transform1.transform.translation.z) / delta_seconds;

    result.angular.z = yawChangeBetweenOrientations(
                            transform1.transform.rotation,
                            transform2.transform.rotation)
                     / delta_seconds;

    return result;
}

double QuadVelocityController::yawChangeBetweenOrientations(
        const geometry_msgs::Quaternion& rotation1,
        const geometry_msgs::Quaternion& rotation2)
{
    // Get the yaw (z axis) rotation from the quanternion
    double current_yaw = yawFromQuaternion(rotation2);
    double last_yaw = yawFromQuaternion(rotation1);

    // Find the difference in yaw
    double yaw_difference = current_yaw - last_yaw;

    // The next two if statements handle if the yaw_difference is large
    //
    // Assumes that we don't have a yaw_difference more than 2pi. We won't
    // since we get all the angles from an atan2 which only return -pi to pi
    //
    // Also assumes that we won't have a rotation difference more than pi
    if (yaw_difference > M_PI)
    {
        yaw_difference = yaw_difference - 2 * M_PI;
    }

    if(yaw_difference < -M_PI)
    {
        yaw_difference = yaw_difference + 2 * M_PI;
    }

    // Finally calculate the yaw change
    return yaw_difference;
}

double QuadVelocityController::yawFromQuaternion(
        const geometry_msgs::Quaternion& rotation)
{
    tf2::Quaternion quaternion;
    tf2::convert(rotation, quaternion);
    
    tf2::Matrix3x3 matrix;
    matrix.getRotation(quaternion);

    double r, p, y;
    matrix.getRPY(r, p, y);
    return y;
}

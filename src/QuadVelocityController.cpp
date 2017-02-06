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

// ROS message headers

using namespace Iarc7Motion;

// Create QuadVelocityController, copy PID settings, initialize all other variables
QuadVelocityController::QuadVelocityController(double thrust_pid[5],
                                               double pitch_pid[5],
                                               double roll_pid[5],
                                               double yaw_pid[5]) :
tfBuffer_(),
tfListener_(tfBuffer_),
throttle_pid_(thrust_pid[0], thrust_pid[1], thrust_pid[2], thrust_pid[3], thrust_pid[4]),
pitch_pid_(pitch_pid[0], pitch_pid[1], pitch_pid[2], pitch_pid[3], pitch_pid[4]),
roll_pid_(roll_pid[0], roll_pid[1], roll_pid[2], roll_pid[3], roll_pid[4]),
yaw_pid_(yaw_pid[0], yaw_pid[1], yaw_pid[2], yaw_pid[3], yaw_pid[4]),
last_transform_stamped_(),
last_yaw_(0.0),
wait_for_velocities_ran_once_(false)
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

    // Pitch and roll velocities are transformed according to the last_yaw_ angle because the incoming
    // target velocities do not take the current yaw into account
    pitch_pid_.setSetpoint(twist.linear.x * std::cos(last_yaw_) + twist.linear.y * std::sin(last_yaw_));

    // Note: Roll is inverted because a positive y velocity means a negative roll by the right hand rule
    roll_pid_.setSetpoint(-(twist.linear.x * -std::sin(last_yaw_) + twist.linear.y * std::cos(last_yaw_)));
}

// Main update, runs all PID calculations and returns a desired uav_command
// Needs to be called at regular intervals in order to keep catching the latest velocities.
bool QuadVelocityController::update(const ros::Time& time,
                                    iarc7_msgs::OrientationThrottleStamped& uav_command)
{
    // Get the current velocity of the quad.
    geometry_msgs::Twist velocities;
    bool success = waitForNewVelocities(velocities);
    if (!success) {
        ROS_WARN("Failed to get current velocities in QuadVelocityController::update");
        return false;
    }

    // Used to temporarily store throttle and angle outputs from PID loops
    double throttle_output;
    double pitch_output;
    double roll_output;
    double yaw_output;

    // Update throttle PID loop
    success = throttle_pid_.update(velocities.linear.z, time, throttle_output);
    if (!success) {
        ROS_WARN("Throttle PID update failed in QuadVelocityController::update");
        return false;
    }

    // Update pitch PID loop
    success = pitch_pid_.update(velocities.linear.x, time, pitch_output);
    if (!success) {
        ROS_WARN("Pitch PID update failed in QuadVelocityController::update");
        return false;
    }

    // Update roll PID loop
    success = roll_pid_.update(-velocities.linear.y, time, roll_output);
    if (!success) {
        ROS_WARN("Roll PID update failed in QuadVelocityController::update");
        return false;
    }

    // Update yaw PID loop
    success = yaw_pid_.update(velocities.angular.z, time, yaw_output);
    if (!success) {
        ROS_WARN("Yaw PID update failed in QuadVelocityController::update");
        return false;
    }

    // Fill in the uav_command's information
    uav_command.header.stamp = time;

    // Simple feedforward using a fixed hover_throttle_ to avoid excessive oscillations from the 
    // PID's I term compensating for there needing to be an  average throttle value at 0 velocity in the z axis.
    uav_command.throttle = throttle_output + hover_throttle_;
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
    ROS_INFO("Vz:       %f Vx:    %f Vy:   %f Vyaw: %f", velocities.linear.z, velocities.linear.x, velocities.linear.y, velocities.angular.z);
    ROS_INFO("Throttle: %f Pitch: %f Roll: %f Yaw:  %f", uav_command.throttle, uav_command.data.pitch, uav_command.data.roll, uav_command.data.yaw);

    return true;
}

// Get a valid transform, blocks until the transform is received or
// the request takes too long and times out
bool QuadVelocityController::waitForTransform(geometry_msgs::TransformStamped& transform) {

    // Catch TransformException exceptions
    try
    {
        ros::Time time_at_start = ros::Time::now();
        ros::Time last_transform_time;
        if(wait_for_velocities_ran_once_)
        {
            last_transform_time = last_transform_stamped_.header.stamp;
        }
        else
        {
            last_transform_time = ros::Time::now();
        }

        // While we aren't supposed to be shutting down
        while (ros::ok())
        {
            // Check for timing out, return failure if so.
            ros::Duration timeout = wait_for_velocities_ran_once_ ?
                                    ros::Duration(MAX_TRANSFORM_WAIT_SECONDS) :
                                    ros::Duration(INITIAL_TRANSFORM_WAIT_SECONDS);

            if (ros::Time::now() > time_at_start + timeout)
            {
                ROS_ERROR_STREAM("Transform timed out ("
                                 << "current time: " << ros::Time::now() << ", "
                                 << "request time: " << time_at_start
                                 << ")");
                return false;
            }
            
            // Check if the transform from map to quad can be made right now
            if(tfBuffer_.canTransform("map", "quad", ros::Time(0)))
            {
                // Get the transform
                transform = tfBuffer_.lookupTransform("map", "quad", ros::Time(0));

                // Check if the transform is newer or as new as the desired time
                if(transform.header.stamp > last_transform_time)
                {
                    return true;
                }
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
        ROS_ERROR("Could not transform map to level_quad: %s",ex.what());
    }

    ROS_ERROR("Exception or ros::ok was false");
    return false;
}

// Gets the velocity from the transformation information
// Blocks while waiting for a new transform to calculate the velocity with
bool QuadVelocityController::waitForNewVelocities(geometry_msgs::Twist& return_velocities)
{

    // Get a brand new transform. Blocks until able to transform at the current time.
    geometry_msgs::TransformStamped transformStamped;
    bool fetched_transform = waitForTransform(transformStamped);

    // Check if the call failed
    if (!fetched_transform)
    {
        ROS_ERROR("Failed to fetch new transform in QuadVelocityController");
        wait_for_velocities_ran_once_ = false;
        return false;
    }

    // Check if the quad velocity controller has not been run once
    // Update all intermediate values and make a recursive call to waitForNewVelocities
    // (meaning another transform will be collected) and a valid velocity can be calculated.
    if(!wait_for_velocities_ran_once_)
    {
        wait_for_velocities_ran_once_ = true;
        last_transform_stamped_ = transformStamped;
        return waitForNewVelocities(return_velocities);
    }

    // Make sure the difference between transforms isn't too much
    // Calculate the lastest time between transforms that is allowed.
    ros::Time latest_time_allowed = last_transform_stamped_.header.stamp
                + ros::Duration(MAX_TRANSFORM_DIFFERENCE_SECONDS);
    // Make the sure the transform isn't didn't come after the last time
    // that was acceptable.
    if (transformStamped.header.stamp > latest_time_allowed)
    {
        ROS_ERROR_STREAM(
            "First available transform came after latest allowed time"
            << " (transform time: " << transformStamped.header.stamp
            << ", latest allowed time: " << latest_time_allowed
            << ")");
        return false;
    }

    // Get the transforms without the stamps for readability
    geometry_msgs::Transform& transform = transformStamped.transform;
    geometry_msgs::Transform& oldTransform = last_transform_stamped_.transform;

    // Get the yaw (z axis) rotation from the quanternion
    double ysqr = transform.rotation.y * transform.rotation.y;
    double t3 = 2.0f * (transform.rotation.w * transform.rotation.z + transform.rotation.x * transform.rotation.y);
    double t4 = 1.0f - 2.0f * (ysqr + transform.rotation.z * transform.rotation.z);  
    double current_yaw = std::atan2(t3, t4);

    // Begin calculation of the velocities

    // Get the time between the two transforms
    ros::Duration delta_seconds = transformStamped.header.stamp - last_transform_stamped_.header.stamp;

    // Calculate world_x and world_y velocities
    double delta = delta_seconds.toSec();
    double world_x_vel = ((transform.translation.x - oldTransform.translation.x) / delta);
    double world_y_vel = ((transform.translation.y - oldTransform.translation.y) / delta);

    // Transform the world_x and world_y velocities according to the quads current heading
    return_velocities.linear.x = world_x_vel * std::cos(current_yaw) + world_y_vel * std::sin(current_yaw);
    return_velocities.linear.y = world_x_vel * -std::sin(current_yaw) + world_y_vel * std::cos(current_yaw);

    // Calculate the world_z velocity, no transform is needed
    return_velocities.linear.z = (transform.translation.z - oldTransform.translation.z) / delta;

    // Find the difference in yaw
    double yaw_difference = current_yaw - last_yaw_;

    // The next two if statements handle if the yaw_difference is large due to gimbal lock
    // Assumes that we don't have a yaw_difference more than 2pi. We won't since we get all the angles
    // from an atan2 which only return -pi to pi
    // Also assumes that we won't have a rotation difference more than pi
    if(yaw_difference > M_PI)
    {
        yaw_difference = yaw_difference - 2 * M_PI;
    }

    if(yaw_difference < -M_PI)
    {
        yaw_difference = yaw_difference + 2 * M_PI;
    }

    // Finally calculate the yaw velocity after correcting for gimbal lock
    return_velocities.angular.z = yaw_difference / delta;

    // Store the old transform
    last_transform_stamped_ = transformStamped;

    // Store yaw
    last_yaw_ = current_yaw;

    return true;
}

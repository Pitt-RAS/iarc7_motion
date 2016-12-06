////////////////////////////////////////////////////////////////////////////
//
// Quad Velocity controller
//
// Implements details involving the throttle controller
//
////////////////////////////////////////////////////////////////////////////

// Associated header
#include "iarc7_motion/QuadVelocityController.hpp"

// Package headers
#include "iarc7_motion/QuadTwistRequestLimiter.hpp"

// ROS message headers
#include "geometry_msgs/Vector3Stamped.h"

using namespace Iarc7Motion;

QuadVelocityController::QuadVelocityController(ros::NodeHandle& nh) :
nh_(nh),  
throttle_pid_(0.0, 0.0, 0.0),
pitch_pid_(0.0, 0.0, 0.0) ,
roll_pid_(0.0, 0.0, 0.0) ,
yaw_pid_(0.0, 0.0, 0.0) ,
tfBuffer_(),
tfListener_(tfBuffer_)
{
    uav_control_ = nh_.advertise<iarc7_msgs::OrientationThrottleStamped>("uav_direction_command", 50);

    // Check for empty uav_control_ as per http://wiki.ros.org/roscpp/Overview/Publishers%20and%20Subscribers
    // section 1
    if(!uav_control_)
    {
        ROS_ASSERT("Could not create uav_control_ publisher");
    }
}

// TODO sanity check these values
void QuadVelocityController::setTargetVelocity(geometry_msgs::Twist twist)
{
    throttle_pid_.setSetpoint(twist.linear.z);
    pitch_pid_.setSetpoint(twist.linear.x);
    roll_pid_.setSetpoint(twist.linear.y);
    yaw_pid_.setSetpoint(twist.angular.z);
}

// Needs to be called at regular intervals in order to keep catching the latest velocities.
void QuadVelocityController::update()
{
    // Get the time delta
    geometry_msgs::Vector3 velocities;
    getVelocities(velocities);
    double time_delta = 0.0;

    // Update all the PID loops
    double throttle_output = throttle_pid_.update(velocities.z, time_delta);
    double pitch_output    = pitch_pid_.update(velocities.x, time_delta);
    double roll_output     = roll_pid_.update(velocities.y, time_delta);
    double yaw_output      = yaw_pid_.update(velocities.y, time_delta);

    // For now publish, should send values to a hard limiter first
    iarc7_msgs::OrientationThrottleStamped uav_command;
    uav_command.throttle = throttle_output;
    uav_command.data.pitch = pitch_output;
    uav_command.data.roll = roll_output;
    uav_command.data.yaw = yaw_output;

    // Print the velocity and throttle information
    ROS_DEBUG("Vz:       %f Vx:    %f Vy:   %f", velocities.z, velocities.x, velocities.y);
    ROS_DEBUG("Throttle: %f Pitch: %f Roll: %f Yaw: %f", uav_command.throttle, uav_command.data.pitch, uav_command.data.roll, uav_command.data.yaw);

    // Limit the uav command
    limitUavCommand(uav_command);

    // Publish the desired angles and throttle
    uav_control_.publish(uav_command);
}

// This is a helper function that will limit a uav command using the twist limiter
// It will eventually no longer be needed when the quad controller is converted to use twists
void QuadVelocityController::limitUavCommand(iarc7_msgs::OrientationThrottleStamped& uav_command)
{
    TwistStamped uav_twist_stamped;
    Twist& uav_twist = uav_twist_stamped.twist;

    // Convert from uav command to twist
    uav_twist_stamped.header.stamp = uav_command.header.stamp;
    uav_twist.linear.z  = uav_command.throttle;
    uav_twist.angular.y = uav_command.data.pitch;
    uav_twist.angular.x = uav_command.data.roll;
    uav_twist.angular.z = uav_command.data.yaw;

    // Limit the twist
    Twist min;
    min.linear.z  = 0.0;
    min.angular.y = -20.0;
    min.angular.x = -20.0;
    min.angular.z = -20.0;

    Twist max;
    min.linear.z  = 100.0;
    min.angular.y = 20.0;
    min.angular.x = 20.0;
    min.angular.z = 20.0;

    Twist max_rate;
    min.linear.z  = 100.0;
    min.angular.y = 20.0;
    min.angular.x = 20.0;
    min.angular.z = 20.0;

    QuadTwistRequestLimiter limiter(min, max, max_rate);
    uav_twist_stamped = limiter.limitTwist(uav_twist_stamped);

    // Copy the twist to the uav command
    uav_command.header.stamp = uav_twist_stamped.header.stamp;
    uav_command.throttle     = uav_twist.linear.z;
    uav_command.data.pitch   = uav_twist.angular.y;
    uav_command.data.roll    = uav_twist.angular.x;
    uav_command.data.yaw     = uav_twist.angular.z;
}


// Waits for the next transform to come in, returns true if velocities are valid
// Has to receive two transforms within the timeout period to consider the velocity valid
bool QuadVelocityController::getVelocities(geometry_msgs::Vector3& return_velocities)
{
    // Holds the last transform received to calculate velocities
    static geometry_msgs::TransformStamped lastTransformStamped;

    // Holds the last valid velocity reading
    static geometry_msgs::Vector3Stamped lastVelocityStamped;

    // Makes sure that we have a lastTransformStamped before returning a valid velocity
    static bool ran_once{false};

    // Can be set to mark a velocity reading invalid
    bool velocities_valid{true};

    // Get the map to level quad transform
    geometry_msgs::TransformStamped transformStamped;

    // Get current ROS time
    ros::Time time = ros::Time::now();

    // Check if we already have a velocity at this time
    if (time == lastVelocityStamped.header.stamp) {
        return_velocities = lastVelocityStamped.vector;
        return true;
    }

    try{
        transformStamped = tfBuffer_.lookupTransform("map", "level_quad", time, ros::Duration(MAX_TRANSFORM_WAIT_SECONDS));

        if(ran_once)
        {
            // Get the time between the two transforms
            ros::Duration delta_seconds = transformStamped.header.stamp - lastTransformStamped.header.stamp;

            if(delta_seconds > ros::Duration(MAX_TRANSFORM_DIFFERENCE_SECONDS))
            {
                ROS_ERROR("Velocities invalid, time between transforms is too high: %f seconds", delta_seconds.toSec());
                velocities_valid = false;
            }

            // Get the transforms with the stamps for readability
            geometry_msgs::Transform& transform = transformStamped.transform;
            geometry_msgs::Transform& oldTransform = lastTransformStamped.transform;

            // Calculate x, y, and z velocity
            double delta = delta_seconds.toSec();

            return_velocities.x = (transform.translation.x - oldTransform.translation.x) / delta;
            return_velocities.y = (transform.translation.y - oldTransform.translation.y) / delta;
            return_velocities.z = (transform.translation.z - oldTransform.translation.z) / delta;

            // Store this as the last valid velocity
            lastVelocityStamped.vector = return_velocities;
            lastVelocityStamped.header.stamp = time;
        }
        else
        {
            velocities_valid = false;
            ran_once = true;
        }

        lastTransformStamped = transformStamped;
    }
    catch (tf2::TransformException& ex){
        ROS_ERROR("Could not transform map to level_quad: %s",ex.what());
        velocities_valid = false;
        ran_once = false;
    }

    return velocities_valid;
}

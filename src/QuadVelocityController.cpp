////////////////////////////////////////////////////////////////////////////
//
// Quad Velocity controller
//
// Implements details involving the throttle controller
//
////////////////////////////////////////////////////////////////////////////

// Associated header
#include "iarc7_motion/QuadVelocityController.hpp"

// ROS message headers
#include "geometry_msgs/Vector3Stamped.h"

using namespace Iarc7Motion;

QuadVelocityController::QuadVelocityController(ros::NodeHandle& nh, double thrust_pid[3], double pitch_pid[3], double roll_pid[3]) :
nh_(nh),  
throttle_pid_(thrust_pid[0], thrust_pid[1], thrust_pid[2], 50, -50),
pitch_pid_(pitch_pid[0], pitch_pid[1], pitch_pid[2], 1, -1) ,
roll_pid_(roll_pid[0], roll_pid[1], roll_pid[2], 1, -1) ,
yaw_pid_(0.0, 0.0, 0.0, 1, -1) ,
tfBuffer_(),
tfListener_(tfBuffer_),
last_time_(0.0)
{

}

// TODO sanity check these values
void QuadVelocityController::setTargetVelocity(geometry_msgs::Twist twist)
{
    throttle_pid_.setSetpoint(twist.linear.z);
    pitch_pid_.setSetpoint(twist.linear.x);
    roll_pid_.setSetpoint(-twist.linear.y);
    yaw_pid_.setSetpoint(twist.angular.z);
}

// Needs to be called at regular intervals in order to keep catching the latest velocities.
iarc7_msgs::OrientationThrottleStamped QuadVelocityController::update(const ros::Time& time)
{
    geometry_msgs::Vector3 velocities;
    getVelocities(velocities);

    // Get the time delta
    if (last_time_ == ros::Time(0)) {
        last_time_ = time;
    }
    double time_delta = (time - last_time_).toSec();
    last_time_ = time;

    // Update all the PID loops
    double throttle_output = throttle_pid_.update(velocities.z, time_delta);
    double pitch_output    = pitch_pid_.update(velocities.x, time_delta);
    double roll_output     = roll_pid_.update(-velocities.y, time_delta);
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

    return uav_command;
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

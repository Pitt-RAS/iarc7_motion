////////////////////////////////////////////////////////////////////////////
//
// Quad Velocity controller
//
// Implements details involving the throttle controller
//
////////////////////////////////////////////////////////////////////////////

#include "QuadVelocityController.hpp"
#include "iarc7_msgs/OrientationThrottleStamped.h"

using namespace Iarc7Motion;

QuadVelocityController::QuadVelocityController(ros::NodeHandle& nh) :
nh_(nh),  
throttle_pid_(0.0, 0.0, 0.0),
pitch_pid_(0.0, 0.0, 0.0) ,
roll_pid_(0.0, 0.0, 0.0) ,
tfBuffer_(),
tfListener_(tfBuffer_)
{

}

void QuadVelocityController::init()
{
    uav_control_ = nh_.advertise<iarc7_msgs::OrientationThrottleStamped>("uav_direction_command", 50);
}

// Needs to be called at regular intervals in order to keep catching the latest velocities.
void QuadVelocityController::update()
{
    // Get the time delta
    geometry_msgs::Vector3 velocities;
    getVelocities(velocities);
    double time_delta = 0.0;

    // Update all the PID loops
    double throttle_output = throttle_pid_.update(velocities.x, time_delta);
    double pitch_output    = throttle_pid_.update(velocities.y, time_delta);
    double roll_output     = throttle_pid_.update(velocities.z, time_delta);

    // For now publish, should send values to a hard limiter first
    iarc7_msgs::OrientationThrottleStamped uav_command;
    uav_command.throttle = throttle_output;
    uav_command.data.pitch = pitch_output;
    uav_command.data.roll = roll_output;
    uav_command.data.yaw = 0;

    // Publish the desired angles and throttle
    uav_control_.publish(uav_command);
}


// Waits for the next transform to come in, returns true if velocities are valid
// Has to receive two transforms within the timeout period to consider the velocity valid
bool QuadVelocityController::getVelocities(geometry_msgs::Vector3& return_velocities)
{
    // Holds the last transform received to calculate velocities
    static geometry_msgs::TransformStamped lastTransformStamped;

    // Makes sure that we have a lastTransformStamped before returning a valid velocity
    static bool ran_once{false};

    // Can be set to mark a velocity reading invalid
    bool velocities_valid{true};

    // Get the map to level quad transform
    geometry_msgs::TransformStamped transformStamped;
    try{
        transformStamped = tfBuffer_.lookupTransform("map", "level_quad", ros::Time::now(), ros::Duration(MAX_TRANSFORM_WAIT_SECONDS));

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
            return_velocities.x = static_cast<double>(transform.translation.x - oldTransform.translation.x) / delta;
            return_velocities.y = static_cast<double>(transform.translation.y - oldTransform.translation.y) / delta;
            return_velocities.z = static_cast<double>(transform.translation.z - oldTransform.translation.z) / delta;
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

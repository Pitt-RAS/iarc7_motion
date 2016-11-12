////////////////////////////////////////////////////////////////////////////
//
// Quad Controller
// Quad Controller
//
// Implements details involving the throttle controller
//
////////////////////////////////////////////////////////////////////////////

#include "QuadController.hpp"
#include "iarc7_msgs/OrientationThrottleStamped.h"

using namespace Iarc7Motion;

QuadController::QuadController(ros::NodeHandle& nh) :
nh_(nh),  
throttle_pid_(0.0, 0.0, 0.0),
pitch_pid_(0.0, 0.0, 0.0) ,
roll_pid_(0.0, 0.0, 0.0) ,
yaw_pid_(0.0, 0.0, 0.0)
{
    interpolation_timer_ = nh_.createTimer(ros::Duration(0.1), &QuadController::update, this);
}

void QuadController::init()
{
    uav_control_ = nh_.advertise<iarc7_msgs::OrientationThrottleStamped>("uav_direction_command", 50);
    altitude_subscriber_ = nh_.subscribe("altitude", 100, &QuadController::setCurrentHeight, this);
}

void QuadController::update(const ros::TimerEvent& time)
{
    double time_delta = (time.current_real - time.last_real).toSec();

    // Handle interpolation, running the pid, and setting the output
    interpolatePositions(time_delta);

    double throttle_output = throttle_pid_.update(interpolated_height_, time_delta);
    double pitch_output = throttle_pid_.update(interpolated_pitch_, time_delta);
    double yaw_output = throttle_pid_.update(interpolated_yaw_, time_delta);
    double roll_output = throttle_pid_.update(interpolated_roll_, time_delta);

    // Handle sending the values here

    iarc7_msgs::OrientationThrottleStamped uav_command;
    uav_command.throttle = throttle_output;
    uav_command.data.pitch = pitch_output;
    uav_command.data.roll = roll_output;
    uav_command.data.yaw = yaw_output;

    // Publish the desired angles and throttle
    uav_control_.publish(uav_command);
}

void QuadController::interpolatePositions(const double time_delta)
{
    // Interpolate our position based on the previous velocity acceleration and position
}

void QuadController::setCurrentHeight(const iarc7_msgs::Float64Stamped::ConstPtr& message)
{
    // Set the current height and all the derivates that go with it
    interpolated_height_ = message->data;
}

void QuadController::setCurrentPitchRollYaw(const double pitch, const double roll, const double yaw)
{
    // Set the current height and all the derivates that go with it
    interpolated_pitch_ = pitch;
    interpolated_roll_ = roll;
    interpolated_yaw_ = yaw;
}


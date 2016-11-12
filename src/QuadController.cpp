////////////////////////////////////////////////////////////////////////////
//
// Quad Controller
// Quad Controller
//
// Implements details involving the throttle controller
//
////////////////////////////////////////////////////////////////////////////

#include "QuadController.hpp"

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

void QuadController::update(const ros::TimerEvent& time)
{
    double time_delta = (time.current_real - time.last_real).toSec();

    // Handle interpolation, running the pid, and setting the output
    interpolatePositions(time_delta);

    double throttle_output = throttle_pid_.update(interpolated_height_, time_delta);
    double pitch_output = throttle_pid_.update(interpolated_pitch_, time_delta);
    double yaw_output = throttle_pid_.update(interpolated_yaw_, time_delta);
    double roll_output = throttle_pid_.update(interpolated_roll_, time_delta);

    // Handle sending the values here or returning them?
}

void QuadController::interpolatePositions(const double time_delta)
{
    // Interpolate our position based on the previous velocity acceleration and position
}

void QuadController::setCurrentHeight(const double height)
{
    // Set the current height and all the derivates that go with it
    interpolated_height_ = height;
}

void QuadController::setCurrentPitchRollYaw(const double pitch, const double roll, const double yaw)
{
    // Set the current height and all the derivates that go with it
    interpolated_pitch_ = pitch;
    interpolated_roll_ = roll;
    interpolated_yaw_ = yaw;
}


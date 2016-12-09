////////////////////////////////////////////////////////////////////////////
//
// FeedForwardPid
//
// Class implement a feed forward PID loop
//
////////////////////////////////////////////////////////////////////////////

#include <cmath>
#include <limits>
#include <ros/ros.h>

#include "iarc7_motion/FeedForwardPid.hpp"

using namespace Iarc7Motion;

FeedForwardPid::FeedForwardPid(double p_gain, double i_gain, double d_gain,
                               double i_accumulator_max, double i_accumulator_min)
    : p_gain_(p_gain),
      i_gain_(i_gain),
      d_gain_(d_gain),
      initialized_(false),
      i_accumulator_(0.0),
      last_current_value_(0.0),
      setpoint_(0.0),
      i_accumulator_max_(i_accumulator_max),
      i_accumulator_min_(i_accumulator_min)
{
    // Nothing to do
}

bool FeedForwardPid::update(double current_value, double time_delta, double& response)
{
    if (!initialized_ && std::isfinite(current_value)) {
        last_current_value_ = current_value;
        initialized_ = true;

        double difference = setpoint_ - current_value;
        double p_term = p_gain_ * difference;
        response = p_term;

        if (!std::isfinite(response)) {
            ROS_WARN("Invalid result from FeedForwardPid::update (response = %f)",
                     response);
            return false;
        } else {
            return true;
        }
    }

    if (!std::isfinite(current_value)) {
        ROS_WARN("Invalid argument to FeedForwardPid::update (current_value = %f)",
                 current_value);
        return false;
    }

    if (!std::isfinite(time_delta) || time_delta <= 0) {
        ROS_WARN("Invalid argument to FeedForwardPid::update (time_delta = %f)",
                 time_delta);
        return false;
    }

    double difference = setpoint_ - current_value;
    double p_term = p_gain_ * difference;
    i_accumulator_ += i_gain_ * difference * time_delta;
    if (i_accumulator_ > i_accumulator_max_) {
        i_accumulator_ = i_accumulator_max_;
    } else if (i_accumulator_ < i_accumulator_min_) {
        i_accumulator_ = i_accumulator_min_;
    }

    double d_term = d_gain_ * (current_value - last_current_value_) / time_delta;

    last_current_value_ = current_value;

    response = p_term + i_accumulator_ - d_term;
    if (!std::isfinite(response)) {
        ROS_WARN("Invalid result from FeedForwardPid::update (response = %f)",
                 response);
        return false;
    } else {
        return true;
    }
}

void FeedForwardPid::setSetpoint(double setpoint)
{
    setpoint_ = setpoint;
}

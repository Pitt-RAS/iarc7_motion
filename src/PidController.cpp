////////////////////////////////////////////////////////////////////////////
//
// PidController
//
// Class implement a PID loop
//
////////////////////////////////////////////////////////////////////////////

#include <cmath>
#include <limits>
#include <ros/ros.h>
#include <boost/algorithm/clamp.hpp>

#include "iarc7_motion/PidController.hpp"

using namespace Iarc7Motion;

PidController::PidController(double p_gain, double i_gain, double d_gain,
                             double i_accumulator_max, double i_accumulator_min,
                             double i_accumulator_enable_threshold)
    : p_gain_(p_gain),
      i_gain_(i_gain),
      d_gain_(d_gain),
      initialized_(false),
      i_accumulator_(0.0),
      last_current_value_(0.0),
      last_time_(0.0),
      setpoint_(0.0),
      i_accumulator_max_(i_accumulator_max),
      i_accumulator_min_(i_accumulator_min),
      i_accumulator_enable_threshold_(i_accumulator_enable_threshold)
{
    // Nothing to do
}

bool PidController::update(double current_value,
                           const ros::Time& time,
                           double& response,
                           double derivative,
                           bool log_debug)
{
    if (time < last_time_) {
        ROS_WARN("Time passed in to PidController is less than the last time.");
        return false;
    }

    if(time == last_time_)
    {
        ROS_WARN("Time passed in to PidController is equal to the last time.");
        return false;
    }

    if (!std::isfinite(current_value)) {
        ROS_WARN("Invalid argument to PidController::update (current_value = %f)",
                 current_value);
        return false;
    }

    if (!std::isnan(derivative) && !std::isfinite(derivative)) {
        ROS_WARN("Invalid argument to PidController::update (derivative = %f)",
                 derivative);
        return false;
    }

    double difference = setpoint_ - current_value;
    double p_term = p_gain_ * difference;
    response = p_term;

    if (!initialized_) {
        initialized_ = true;
    } else {
        double time_delta = (time - last_time_).toSec();

        if (std::abs(difference) < i_accumulator_enable_threshold_) {
            i_accumulator_ += i_gain_ * difference * time_delta;

            boost::algorithm::clamp(i_accumulator_, i_accumulator_min_, i_accumulator_max_);
        }

        response += i_accumulator_;

        // Check if we were passed a derivative or not
        if (std::isnan(derivative)) {
            derivative = (current_value - last_current_value_) / time_delta;
        }

        double d_term = d_gain_ * derivative;
        response -= d_term;

        if (log_debug) {
            ROS_WARN("p %f i %f d %f", p_term, i_accumulator_, d_term);
        }
    }

    last_current_value_ = current_value;
    last_time_ = time;

    if (!std::isfinite(response)) {
        ROS_WARN("Invalid result from PidController::update (response = %f)",
                 response);
        return false;
    } else {
        return true;
    }
}

void PidController::setSetpoint(double setpoint)
{
    setpoint_ = setpoint;
}

void PidController::resetAccumulator()
{
    i_accumulator_ = 0;
}

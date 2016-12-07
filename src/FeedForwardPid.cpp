////////////////////////////////////////////////////////////////////////////
//
// FeedForwardPid
//
// Class implement a feed forward PID loop
//
////////////////////////////////////////////////////////////////////////////

#include <limits>

#include "iarc7_motion/FeedForwardPid.hpp"

using namespace Iarc7Motion;

FeedForwardPid::FeedForwardPid(double p_gain, double i_gain, double d_gain,
                               double i_accumulator_max, double i_accumulator_min)
    : p_gain_(p_gain),
      i_gain_(i_gain),
      d_gain_(d_gain),
      setpoint_(0.0),
      i_accumulator_(0.0),
      last_current_value_(0.0),
      i_accumulator_max_(i_accumulator_max),
      i_accumulator_min_(i_accumulator_min)
{
    // Nothing to do
}

double FeedForwardPid::update(double current_value, double time_delta)
{
    if (time_delta == 0) {
        time_delta = std::numeric_limits<double>::min();
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

    double response = p_term + i_accumulator_ - d_term;

    last_current_value_ = current_value;

    return response;
}

void FeedForwardPid::setSetpoint(double setpoint)
{
    setpoint_ = setpoint;
}

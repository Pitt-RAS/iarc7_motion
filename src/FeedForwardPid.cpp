////////////////////////////////////////////////////////////////////////////
//
// FeedForwardPid
//
// Class implement a feed forward PID loop
//
////////////////////////////////////////////////////////////////////////////

#include "FeedForwardPid.hpp"

using namespace Iarc7Motion;

FeedForwardPid::FeedForwardPid(double p_gain, double i_gain, double d_gain) : p_gain_(p_gain), i_gain_(i_gain), d_gain_(d_gain), setpoint_(0.0)
{
    // Nothing to do
}

double FeedForwardPid::update(double current_value, double time_delta)
{

    static double i_accumulator{0.0};
    static double last_current_value{0.0};

    double difference = current_value - setpoint_;
    double p_term = p_gain_ * difference;
    i_accumulator += i_gain_ * difference;
    double d_term = d_gain_ * (current_value - last_current_value);

    double response = p_term + i_accumulator - d_term;

    return response;
}

void FeedForwardPid::setSetpoint(double setpoint)
{
    setpoint_ = setpoint;
}

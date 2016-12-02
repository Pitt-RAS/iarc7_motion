#ifndef FEED_FORWARD_PID_HPP
#define FEED_FORWARD_PID_HPP

////////////////////////////////////////////////////////////////////////////
//
// FeedForwardPid
//
// Class implement a feed forward PID loop
//
////////////////////////////////////////////////////////////////////////////

namespace Iarc7Motion
{

    class FeedForwardPid
    {
    public:

        FeedForwardPid(double p_gain, double i_gain, double d_gain);

        FeedForwardPid() = delete;
        ~FeedForwardPid() = default;

        // Don't allow the copy constructor or assignment.
        FeedForwardPid(const FeedForwardPid& rhs) = delete;
        FeedForwardPid& operator=(const FeedForwardPid& rhs) = delete;

        void setSetpoint(double setpoint);
        double update(double current_value, double time_delta);

    private:
        const double p_gain_;
        const double i_gain_;
        const double d_gain_;

        double setpoint_;

    };

}

#endif

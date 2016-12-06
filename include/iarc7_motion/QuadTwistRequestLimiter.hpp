////////////////////////////////////////////////////////////////////////////
//
// Twist Limiter
//
// Limits the rate of change of a twist and min/max values
//
////////////////////////////////////////////////////////////////////////////

#ifndef QUAD_TWIST_LIMITER_HPP
#define QUAD_TWIST_LIMITER_HPP

#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TwistStamped.h"

using geometry_msgs::TwistStamped;
using geometry_msgs::Twist;

namespace Iarc7Motion
{

    class QuadTwistRequestLimiter
    {
    public:
        QuadTwistRequestLimiter() = delete;

        QuadTwistRequestLimiter(Twist minTwist, Twist maxTwist, Twist maxChange);

        ~QuadTwistRequestLimiter() = default;

        // Don't allow the copy constructor or assignment.
        QuadTwistRequestLimiter(const QuadTwistRequestLimiter& rhs) = delete;
        QuadTwistRequestLimiter& operator=(const QuadTwistRequestLimiter& rhs) = delete;

        TwistStamped limitTwist(const TwistStamped& twist);

    private:

        static double velocityLimit(double start, double end, double max, ros::Duration& delta, char const * axis);
        static double minLimit(double request, double min, char const * axis);
        static double maxLimit(double request, double max, char const * axis);

        Twist minTwist_;
        Twist maxTwist_;
        Twist maxTwistChange_;
    };
} // End namespace Iarc7Motion

#endif // QUAD_TWIST_LIMITER_HPP

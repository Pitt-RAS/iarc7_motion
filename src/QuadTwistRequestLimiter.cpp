////////////////////////////////////////////////////////////////////////////
//
// Twist Limiter
//
// Limits the rate of change of a twist and min/max values
//
////////////////////////////////////////////////////////////////////////////

#include "iarc7_motion/QuadTwistRequestLimiter.hpp"
#include <cmath>

using namespace Iarc7Motion;
QuadTwistRequestLimiter::QuadTwistRequestLimiter(Twist minTwist, Twist maxTwist, Twist maxChange) :
minTwist_(minTwist) ,
maxTwist_(maxTwist) ,
maxTwistChange_(maxChange)
{

}

// TODO sanity check these values
void QuadTwistRequestLimiter::limitTwist(TwistStamped& input_twist)
{
    static TwistStamped last_twist;
    static bool run_once{false};

    if(!run_once)
    {
        last_twist = input_twist;
        run_once = true;

        // Return empty TwistStamped
        TwistStamped return_twist;
        return_twist.header.stamp = input_twist.header.stamp;
        input_twist = return_twist;
        return;
    }

    // Limit the rates
    ros::Duration delta = input_twist.header.stamp - last_twist.header.stamp;
    velocityLimit(input_twist.twist.linear.z,  last_twist.twist.linear.z,  maxTwistChange_.linear.z,  delta, "Thrust");
    // If X extends towards the front of the quad than rotating around the X axis is pitch
    velocityLimit(input_twist.twist.angular.y, last_twist.twist.angular.y, maxTwistChange_.angular.y, delta, "Pitch");
    // If X extends towards the front of the quad than rotating around the X axis is roll
    velocityLimit(input_twist.twist.angular.x, last_twist.twist.angular.x, maxTwistChange_.angular.x, delta, "Roll");
    // If X extends towards the front of the quad than rotating around the Z axis is yaw
    velocityLimit(input_twist.twist.angular.z, last_twist.twist.angular.z, maxTwistChange_.angular.z, delta, "Yaw");

    // Limit the max
    maxLimit(input_twist.twist.linear.z,  maxTwist_.linear.z,  "Thrust");
    maxLimit(input_twist.twist.angular.y, maxTwist_.angular.y, "Pitch");
    maxLimit(input_twist.twist.angular.x, maxTwist_.angular.x, "Roll");
    maxLimit(input_twist.twist.angular.z, maxTwist_.angular.z, "Yaw");

    // Limit the min
    minLimit(input_twist.twist.linear.z,  minTwist_.linear.z,  "Thrust");
    minLimit(input_twist.twist.angular.y, minTwist_.angular.y, "Pitch");
    minLimit(input_twist.twist.angular.x, minTwist_.angular.x, "Roll");
    minLimit(input_twist.twist.angular.z, minTwist_.angular.z, "Yaw");
}

void QuadTwistRequestLimiter::velocityLimit(double& request, const double old, const double max, const ros::Duration& delta, char const * axis)
{
    double velocity = (request - old) / delta.toSec();

    if(abs(velocity) > max)
    {
        // result = (sign of velocity) * max_velocity + start
        double result = (((velocity > 0) - (velocity < 0)) * max) + old;
        ROS_WARN("%s rate limit reached, requested: %f allowed: %f", axis, result, old);
        request = result;
    }
}

void QuadTwistRequestLimiter::maxLimit(double& request, const double max, char const * axis)
{
    if(request > max)
    {
        ROS_WARN("%s max value exceeded, requested: %f allowed: %f", axis, request, max);
        request = max;
    }
}

void QuadTwistRequestLimiter::minLimit(double& request, const double min, char const * axis)
{
    if(request < min)
    {
        ROS_WARN("%s min value exceeded, requested: %f allowed: %f", axis, request, min);
        request = min;
    }
}
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
TwistStamped QuadTwistRequestLimiter::limitTwist(const TwistStamped& twist)
{
    static TwistStamped last_twist;
    static bool run_once{false};

    TwistStamped returnTwist;

    returnTwist.header.stamp = twist.header.stamp;

    if(!run_once)
    {
        last_twist = twist;
        run_once = true;
        // Return empty TwistStamped
        return TwistStamped();
    }

    // Limit the rates
    ros::Duration delta = twist.header.stamp - last_twist.header.stamp;
    returnTwist.twist.linear.z  = velocityLimit(last_twist.twist.linear.z,  twist.twist.linear.z,  maxTwistChange_.linear.z,  delta, "Thrust");
    // If X extends towards the front of the quad than rotating around the X axis is pitch
    returnTwist.twist.angular.y = velocityLimit(last_twist.twist.angular.y, twist.twist.angular.y, maxTwistChange_.angular.y, delta, "Pitch");
    // If X extends towards the front of the quad than rotating around the X axis is roll
    returnTwist.twist.angular.x = velocityLimit(last_twist.twist.angular.x, twist.twist.angular.x, maxTwistChange_.angular.x, delta, "Roll");
    // If X extends towards the front of the quad than rotating around the Z axis is yaw
    returnTwist.twist.angular.z = velocityLimit(last_twist.twist.angular.z, twist.twist.angular.z, maxTwistChange_.angular.z, delta, "Yaw");

    // Limit the max
    returnTwist.twist.linear.z  = maxLimit(returnTwist.twist.linear.z,  maxTwist_.linear.z,  "Thrust");
    returnTwist.twist.angular.y = maxLimit(returnTwist.twist.angular.y, maxTwist_.angular.y, "Pitch");
    returnTwist.twist.angular.x = maxLimit(returnTwist.twist.angular.x, maxTwist_.angular.x, "Roll");
    returnTwist.twist.angular.z = maxLimit(returnTwist.twist.angular.z, maxTwist_.angular.z, "Yaw");

    // Limit the min
    returnTwist.twist.linear.z  = minLimit(returnTwist.twist.linear.z,  minTwist_.linear.z,  "Thrust");
    returnTwist.twist.angular.y = minLimit(returnTwist.twist.angular.y, minTwist_.angular.y, "Pitch");
    returnTwist.twist.angular.x = minLimit(returnTwist.twist.angular.x, minTwist_.angular.x, "Roll");
    returnTwist.twist.angular.z = minLimit(returnTwist.twist.angular.z, minTwist_.angular.z, "Yaw");

    return returnTwist;
}

double QuadTwistRequestLimiter::velocityLimit(double start, double end, double max, ros::Duration& delta, char const * axis)
{
    double velocity = (end - start) / delta.toSec();
    double result{0.0};

    if(abs(velocity) > max)
    {
        // Multiply max by the sign of velocity
        result = (max * ((velocity > 0) - (velocity < 0))) + start;
        ROS_ERROR("%s rate limit reached, requested: %f given: %f", axis, result, end);
    }
    else
    {  
        result = end;
    }

    return result;
}

double QuadTwistRequestLimiter::maxLimit(double request, double max, char const * axis)
{
    if(request > max)
    {
        ROS_ERROR("%s max value exceeded, requested: %f given: %f", axis, request, max);
        return max;
    }
    
    return request;
}

double QuadTwistRequestLimiter::minLimit(double request, double min, char const * axis)
{
    if(request < min)
    {
        ROS_ERROR("%s min value exceeded, requested: %f given: %f", axis, request, min);
        return min;
    }
    
    return request;
}
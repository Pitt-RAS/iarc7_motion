////////////////////////////////////////////////////////////////////////////
//
// Twist Limiter
//
// Limits the min and max values of an input twist and the max rate of change
// of the twist when compared to the last twist passed in.
//
////////////////////////////////////////////////////////////////////////////

#include "iarc7_motion/QuadTwistRequestLimiter.hpp"
#include <cmath>

using namespace Iarc7Motion;

// Construct the object
QuadTwistRequestLimiter::QuadTwistRequestLimiter(Twist minTwist, Twist maxTwist, Twist maxChange) :
minTwist_(minTwist) ,
maxTwist_(maxTwist) ,
maxTwistChange_(maxChange),
run_once_(false)
{

}

// Calls the appropriate limiting functions on each value in the twist.
void QuadTwistRequestLimiter::limitTwist(TwistStamped& input_twist)
{
    // If we haven't run once store the input twist and return a twist full of zeros.
    if(!run_once_)
    {
        last_twist_ = input_twist;
        run_once_ = true;

        // Return empty TwistStamped
        TwistStamped return_twist;
        return_twist.header.stamp = input_twist.header.stamp;
        input_twist = return_twist;
        return;
    }

    // Get the time between the current twist and the last twist used for rate limiting
    ros::Duration delta = input_twist.header.stamp - last_twist_.header.stamp;

    // The next four velocity limit commands limit the max rate of change between the last twist and the current twist
    velocityLimit(input_twist.twist.linear.z,  last_twist_.twist.linear.z,  maxTwistChange_.linear.z,  delta, "Thrust");

    // If X extends towards the front of the quad than rotating around the Y axis is pitch
    velocityLimit(input_twist.twist.angular.y, last_twist_.twist.angular.y, maxTwistChange_.angular.y, delta, "Pitch");
    // If X extends towards the front of the quad than rotating around the X axis is roll
    velocityLimit(input_twist.twist.angular.x, last_twist_.twist.angular.x, maxTwistChange_.angular.x, delta, "Roll");
    // If X extends towards the front of the quad than rotating around the Z axis is yaw
    velocityLimit(input_twist.twist.angular.z, last_twist_.twist.angular.z, maxTwistChange_.angular.z, delta, "Yaw");

    // Limit the max of each component of each twist
    maxLimit(input_twist.twist.linear.z,  maxTwist_.linear.z,  "Thrust");
    maxLimit(input_twist.twist.angular.y, maxTwist_.angular.y, "Pitch");
    maxLimit(input_twist.twist.angular.x, maxTwist_.angular.x, "Roll");
    maxLimit(input_twist.twist.angular.z, maxTwist_.angular.z, "Yaw");

    // Limit the min of each component of each twist
    minLimit(input_twist.twist.linear.z,  minTwist_.linear.z,  "Thrust");
    minLimit(input_twist.twist.angular.y, minTwist_.angular.y, "Pitch");
    minLimit(input_twist.twist.angular.x, minTwist_.angular.x, "Roll");
    minLimit(input_twist.twist.angular.z, minTwist_.angular.z, "Yaw");

    last_twist_ = input_twist;
}

// Limit the max rate of change. Takes in a requested value, the last value, the max rate of change, and the time between
void QuadTwistRequestLimiter::velocityLimit(double& request, const double old, const double max, const ros::Duration& delta, char const * axis)
{
    // Find the rate of change between the current and last value
    double velocity = (request - old) / delta.toSec();

    // If the rate of change is too high
    if(std::abs(velocity) > max)
    {
        // Limit the input value according to v*dt+x with 'x' being the old value
        // 'v' being the max rate of change and dt being the current difference in time
        double result = ((velocity > 0 ? 1 : -1) * max * delta.toSec()) + old;
        ROS_WARN("%s rate limit reached, requested: %f allowed: [%f, %f] result: %f",
                 axis, request, old - max * delta.toSec(), old + max * delta.toSec(), result);
        // We have to copy result to request so that we can request in the ROS_WARN above
        request = result;
    }
}

// Limit an input value to a max value
void QuadTwistRequestLimiter::maxLimit(double& request, const double max, char const * axis)
{
    if(request > max)
    {
        ROS_WARN("%s max value exceeded, requested: %f allowed: %f", axis, request, max);
        request = max;
    }
}

// Limit an input value to a min value
void QuadTwistRequestLimiter::minLimit(double& request, const double min, char const * axis)
{
    if(request < min)
    {
        ROS_WARN("%s min value exceeded, requested: %f allowed: %f", axis, request, min);
        request = min;
    }
}

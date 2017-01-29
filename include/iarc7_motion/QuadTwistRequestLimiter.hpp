////////////////////////////////////////////////////////////////////////////
//
// Twist Limiter
//
// Limits the min and max values of an input twist and the max rate of change
// of the twist when compared to the last twist passed in.
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
    // We don't need the default constructor
    QuadTwistRequestLimiter() = delete;

    // Require that min, max, and max rate be passed on on object creation
    QuadTwistRequestLimiter(Twist minTwist, Twist maxTwist, Twist maxChange);

    ~QuadTwistRequestLimiter() = default;

    // Don't allow the copy constructor or assignment.
    QuadTwistRequestLimiter(const QuadTwistRequestLimiter& rhs) = delete;
    QuadTwistRequestLimiter& operator=(const QuadTwistRequestLimiter& rhs) = delete;

    // Limits the input twist. The passed in twist is modified according to the limiting rules
    void limitTwist(TwistStamped& input_twist);

private:

    // Static method to limit the max rate of change between two numbers based on a past in delta time
    static void velocityLimit(double& request, const double old, const double max, const ros::Duration& delta, char const * axis);
    // Static method to limit the minimum value of a number
    static void minLimit(double& request, const double min, char const * axis);
    // Static method to limit the maximum value of a number
    static void maxLimit(double& request, const double max, char const * axis);

    // These three member variables contain the min, max, and max rate of change settings
    Twist minTwist_;
    Twist maxTwist_;
    Twist maxTwistChange_;

    // Hold the last twist for use when limiting the rate of change
    TwistStamped last_twist_;

    // Used to know is we have a last_twist_ to use
    bool run_once_;
};

} // End namespace Iarc7Motion

#endif // QUAD_TWIST_LIMITER_HPP

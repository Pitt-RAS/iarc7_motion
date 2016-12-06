////////////////////////////////////////////////////////////////////////////
//
// LowLevelMotionController
//
// Class to implement control of the quads movement
//
////////////////////////////////////////////////////////////////////////////

#include <ros/ros.h>

#include "iarc7_motion/AccelerationPlanner.hpp"
#include "iarc7_motion/QuadVelocityController.hpp"
#include "iarc7_motion/QuadTwistRequestLimiter.hpp"

#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TwistStamped.h"

using namespace Iarc7Motion;
using geometry_msgs::TwistStamped;
using geometry_msgs::Twist;

// This is a helper function that will limit a uav command using the twist limiter
// It will eventually no longer be needed when the quad velocity controller is converted to use twists
void limitUavCommand(QuadTwistRequestLimiter& limiter, iarc7_msgs::OrientationThrottleStamped& uav_command)
{
    TwistStamped uav_twist_stamped;
    Twist& uav_twist = uav_twist_stamped.twist;

    // Convert from uav command to twist
    uav_twist_stamped.header.stamp = uav_command.header.stamp;
    uav_twist.linear.z  = uav_command.throttle;
    uav_twist.angular.y = uav_command.data.pitch;
    uav_twist.angular.x = uav_command.data.roll;
    uav_twist.angular.z = uav_command.data.yaw;

    limiter.limitTwist(uav_twist_stamped);

    // Copy the twist to the uav command
    uav_command.header.stamp = uav_twist_stamped.header.stamp;
    uav_command.throttle     = uav_twist.linear.z;
    uav_command.data.pitch   = uav_twist.angular.y;
    uav_command.data.roll    = uav_twist.angular.x;
    uav_command.data.yaw     = uav_twist.angular.z;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Low_Level_Motion_Control");

    ROS_INFO("Low_Level_Motion_Control begin");

    ros::NodeHandle nh;

    QuadVelocityController quadController(nh);

    AccelerationPlanner<QuadVelocityController> accelerationPlanner(nh, quadController);

    ros::Publisher uav_control_ = nh.advertise<iarc7_msgs::OrientationThrottleStamped>("uav_direction_command", 50);

    // Check for empty uav_control_ as per http://wiki.ros.org/roscpp/Overview/Publishers%20and%20Subscribers
    // section 1
    if(!uav_control_)
    {
        ROS_ASSERT("Could not create uav_control_ publisher");
    }

    // Limit the twist, this will be cleaned up when we switch to rosparam
    Twist min;
    min.linear.z  = 0.0;
    min.angular.y = -20.0;
    min.angular.x = -20.0;
    min.angular.z = -20.0;

    Twist max;
    min.linear.z  = 100.0;
    min.angular.y = 20.0;
    min.angular.x = 20.0;
    min.angular.z = 20.0;

    Twist max_rate;
    min.linear.z  = 100.0;
    min.angular.y = 20.0;
    min.angular.x = 20.0;
    min.angular.z = 20.0;

    QuadTwistRequestLimiter limiter(min, max, max_rate);

    while(ros::ok())
    {
        iarc7_msgs::OrientationThrottleStamped uav_command = quadController.update();

        // Limit the uav command
        limitUavCommand(limiter, uav_command);

        // Publish the desired angles and throttle
        uav_control_.publish(uav_command);

        ros::spinOnce();
    }

    // All is good.
    return 0;
}

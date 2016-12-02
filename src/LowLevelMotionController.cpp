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

using namespace Iarc7Motion;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Low_Level_Motion_Control");

    ROS_INFO("Low_Level_Motion_Control begin");

    ros::NodeHandle nh;

    QuadVelocityController quadController(nh);

    AccelerationPlanner<QuadVelocityController> accelerationPlanner(nh, quadController);

    while(ros::ok())
    {
        quadController.update();
        ros::spinOnce();
    }

    // All is good.
    return 0;
}

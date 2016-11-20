////////////////////////////////////////////////////////////////////////////
//
// LowLevelMotionController
//
// Class to implement control of the quads movement
//
////////////////////////////////////////////////////////////////////////////

#include <ros/ros.h>
#include "QuadVelocityController.hpp"

using namespace Iarc7Motion;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Low_Level_Motion_Control");

    ROS_INFO("Low_Level_Motion_Control begin");

    ros::NodeHandle nh;

    QuadVelocityController quadController(nh);

    quadController.init();

    // All is good.
    return 0;
}

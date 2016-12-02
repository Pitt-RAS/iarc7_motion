////////////////////////////////////////////////////////////////////////////
//
// Quad Velocity Controller
//
// Implements details involving the throttle controller
//
////////////////////////////////////////////////////////////////////////////

#ifndef QUAD_VELOCITY_CONTROLLER_H
#define QUAD_VELOCITY_CONTROLLER_H

#include <ros/ros.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3.h>
#include "FeedForwardPid.hpp"
#include "iarc7_msgs/Float64Stamped.h"
#include <tf2_ros/transform_listener.h>

namespace Iarc7Motion
{

    class QuadVelocityController
    {
    public:
        QuadVelocityController() = delete;

        QuadVelocityController(ros::NodeHandle& nh);

        ~QuadVelocityController() = default;

        // Don't allow the copy constructor or assignment.
        QuadVelocityController(const QuadVelocityController& rhs) = delete;
        QuadVelocityController& operator=(const QuadVelocityController& rhs) = delete;

        void update();

    private:

        bool getVelocities(geometry_msgs::Vector3& return_velocities);

        ros::NodeHandle& nh_;

        ros::Publisher uav_control_;

        tf2_ros::Buffer tfBuffer_;
        tf2_ros::TransformListener tfListener_;

        FeedForwardPid throttle_pid_;
        FeedForwardPid pitch_pid_;
        FeedForwardPid roll_pid_;
        FeedForwardPid yaw_pid_;

        static constexpr double MAX_TRANSFORM_WAIT_SECONDS{1.0};
        static constexpr double MAX_TRANSFORM_DIFFERENCE_SECONDS{0.3};
    };

}

#endif // QUAD_VELOCITY_CONTROLLER_H

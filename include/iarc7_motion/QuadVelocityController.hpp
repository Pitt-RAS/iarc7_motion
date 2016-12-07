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
#include "iarc7_motion/FeedForwardPid.hpp"
#include "tf2_ros/transform_listener.h"

#include "geometry_msgs/Transform.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Vector3.h"
#include "iarc7_msgs/Float64Stamped.h"
#include "iarc7_msgs/OrientationThrottleStamped.h"

namespace Iarc7Motion
{

    class QuadVelocityController
    {
    public:
        QuadVelocityController() = delete;

        QuadVelocityController(ros::NodeHandle& nh, double thrust_pid[3], double pitch_pid[3], double roll_pid[3]);

        ~QuadVelocityController() = default;

        // Don't allow the copy constructor or assignment.
        QuadVelocityController(const QuadVelocityController& rhs) = delete;
        QuadVelocityController& operator=(const QuadVelocityController& rhs) = delete;

        iarc7_msgs::OrientationThrottleStamped update(const ros::Time& time);

        void setTargetVelocity(geometry_msgs::Twist twist);

    private:

        bool getVelocities(geometry_msgs::Vector3& return_velocities);

        static void limitUavCommand(iarc7_msgs::OrientationThrottleStamped& uav_command);

        ros::NodeHandle& nh_;

        tf2_ros::Buffer tfBuffer_;
        tf2_ros::TransformListener tfListener_;

        FeedForwardPid throttle_pid_;
        FeedForwardPid pitch_pid_;
        FeedForwardPid roll_pid_;
        FeedForwardPid yaw_pid_;

        ros::Time last_time_;

        double hover_throttle_;

        static constexpr double MAX_TRANSFORM_WAIT_SECONDS{1.0};
        static constexpr double MAX_TRANSFORM_DIFFERENCE_SECONDS{0.3};
    };

}

#endif // QUAD_VELOCITY_CONTROLLER_H

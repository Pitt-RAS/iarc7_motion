////////////////////////////////////////////////////////////////////////////
//
// Quad Controller
//
// Implements details involving the throttle controller
//
////////////////////////////////////////////////////////////////////////////

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

        void init();

    private:
        void update();

        bool getVelocities(geometry_msgs::Vector3& return_velocities);

        ros::NodeHandle& nh_;

        ros::Publisher uav_control_;

        tf2_ros::Buffer tfBuffer_;
        tf2_ros::TransformListener tfListener_;

        FeedForwardPid throttle_pid_;
        FeedForwardPid pitch_pid_;
        FeedForwardPid roll_pid_;

        static const uint32_t NANO_SECONDS_IN_SECOND = 1'000'000U;
        static const double MAX_TRANSFORM_WAIT_SECONDS = 1.0;
    };

}

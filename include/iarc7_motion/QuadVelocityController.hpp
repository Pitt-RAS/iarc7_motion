////////////////////////////////////////////////////////////////////////////
//
// Quad Velocity Controller
//
// Accepts a target velocity and uses 4 PID loops (pitch, yaw, roll, thrust)
// To attempt to hit the target velocity.
//
////////////////////////////////////////////////////////////////////////////

#ifndef QUAD_VELOCITY_CONTROLLER_H
#define QUAD_VELOCITY_CONTROLLER_H

#include <ros/ros.h>
#include "iarc7_motion/PidController.hpp"
#include "tf2_ros/transform_listener.h"

#include "geometry_msgs/Transform.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "iarc7_msgs/Float64Stamped.h"
#include "iarc7_msgs/OrientationThrottleStamped.h"
#include "nav_msgs/Odometry.h"

namespace Iarc7Motion
{

    class QuadVelocityController
    {
    public:
        QuadVelocityController() = delete;

        // Require that PID parameters are passed in upon class creation
        QuadVelocityController(double thrust_pid[5],
                               double pitch_pid[5],
                               double roll_pid[5],
                               double hover_throttle,
                               ros::NodeHandle& nh,
                               ros::NodeHandle& private_nh);

        ~QuadVelocityController() = default;

        // Don't allow the copy constructor or assignment.
        QuadVelocityController(const QuadVelocityController& rhs) = delete;
        QuadVelocityController& operator=(const QuadVelocityController& rhs) = delete;

        // Set a target velocity for the PID loops
        void setTargetVelocity(geometry_msgs::Twist twist);

        // Require checking of the returned value.
        // Used to update all PID loops according to a time delta that is passed in.
        // Return the uav_command it wants sent to the flight controller.
        bool __attribute__((warn_unused_result)) update(
            const ros::Time& time,
            iarc7_msgs::OrientationThrottleStamped& uav_command);

        /// Waits until this object is ready to begin normal operation
        bool __attribute__((warn_unused_result)) waitUntilReady();

    private:

        /// Waits until a transform is available at time or later, returns
        /// true on success, returns a transform using the passed in reference to
        /// a transform.
        bool __attribute__((warn_unused_result)) getTransformAtTime(
                geometry_msgs::TransformStamped& transform,
                const ros::Time& time,
                const ros::Duration& timeout) const;

        /// Blocks waiting for a message to come in at the requested time,
        /// returns true if velocities are valid.  The returned velocity is in
        /// the level_quad frame.
        bool __attribute__((warn_unused_result)) getVelocityAtTime(
                geometry_msgs::Vector3& velocity,
                const ros::Time& time,
                const ros::Duration& timeout);

        /// Callback to handle messages on odometry/filtered
        void odometryCallback(const nav_msgs::Odometry& msg);

        /// Comparator that compares a message by its timestamp
        template<class MsgType>
        static bool timeVsMsgStampedComparator(const ros::Time& time,
                                               const MsgType& msg)
        {
            return time < msg.header.stamp;
        }

        /// Looks at setpoint_ and sets our pid controller setpoints accordinly
        /// based on our current yaw
        void updatePidSetpoints(double current_yaw);

        /// Blocks while waiting until we have an odometry message at time
        /// or both before and after time
        ///
        /// Returns false if the operation times out or we get a message after
        /// time but not before time
        ///
        /// Precondition: odometry_msg_queue_ must contain a message with
        /// msg.header.stamp < time
        ///
        /// Postcondition: odometry_msg_queue_ will contain a message with
        /// msg.header.stamp >= time and a message with
        /// msg.header.stamp <= time
        bool __attribute__((warn_unused_result)) waitForOdometryAtTime(
                const ros::Time& time,
                const ros::Duration& timeout);

        double yawFromQuaternion(const geometry_msgs::Quaternion& rotation);

        // The three PID controllers
        PidController throttle_pid_;
        PidController pitch_pid_;
        PidController roll_pid_;

        // TF listener objects
        tf2_ros::Buffer tfBuffer_;
        const tf2_ros::TransformListener tfListener_;

        // The subscriber for /odometry/filtered
        const ros::Subscriber odometry_subscriber_;

        // Queue of received odometry messages
        std::vector<nav_msgs::Odometry> odometry_msg_queue_;

        // The current setpoint
        geometry_msgs::Twist setpoint_;

        // A fudge feed forward value used for more stable hovering
        double hover_throttle_;

        // Last time an update was successful
        ros::Time last_update_time_;

        // Max allowed timeout waiting for first velocity and transform
        const ros::Duration max_initial_wait_;

        // Max allowed timeout waiting for velocities and transforms
        const ros::Duration max_wait_;
    };
}

#endif // QUAD_VELOCITY_CONTROLLER_H

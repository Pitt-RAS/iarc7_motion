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
#include "iarc7_motion/ThrustModel.hpp"
#include "ros_utils/SafeTransformWrapper.hpp"

#include "geometry_msgs/AccelWithCovarianceStamped.h"
#include "geometry_msgs/Transform.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "iarc7_msgs/Float64Stamped.h"
#include "iarc7_msgs/OrientationThrottleStamped.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Float32.h"

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
                               const ThrustModel& thrust_model,
                               const ros::Duration& battery_timeout,
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
        /// Blocks waiting for a message to come in at the requested time,
        /// returns true if the result is valid.
        template<class StampedMsgType, typename OutType>
        bool __attribute__((warn_unused_result)) getInterpolatedMsgAtTime(
                typename std::vector<StampedMsgType>& queue,
                OutType& out,
                const ros::Time& time,
                const ros::Duration& allowed_time_offset,
                const ros::Duration& timeout,
                const std::function<OutType(const StampedMsgType&)>& extractor) const;

        /// Callback to handle messages on odometry/filtered
        template<class StampedMsgType>
        void msgCallback(const typename StampedMsgType::ConstPtr& msg,
                         std::vector<StampedMsgType>& queue);

        /// Comparator that compares a message by its timestamp
        template<class MsgType>
        static bool timeVsMsgStampedComparator(const MsgType& msg,
                                               const ros::Time& time)
        {
            return msg.header.stamp < time;
        }

        /// Looks at setpoint_ and sets our pid controller setpoints accordinly
        /// based on our current yaw
        void updatePidSetpoints(double current_yaw);

        /// Blocks while waiting until we have an message in the queue at time
        /// or both before and after time
        ///
        /// Returns false if the operation times out or the precondition is not
        /// satisfied
        ///
        /// Precondition: the queue must contain a message with
        /// msg.header.stamp < last_update_time_
        ///
        /// Postcondition: the queue will contain a message with
        /// msg.header.stamp >= time and a message with
        /// msg.header.stamp < last_update_time_
        template<class StampedMsgType>
        bool __attribute__((warn_unused_result)) waitForMsgAtTime(
                const std::vector<StampedMsgType>& queue,
                const ros::Time& time,
                const ros::Duration& timeout) const;

        double yawFromQuaternion(const geometry_msgs::Quaternion& rotation);

        // The three PID controllers
        PidController throttle_pid_;
        PidController pitch_pid_;
        PidController roll_pid_;

        ThrustModel thrust_model_;

        ros_utils::SafeTransformWrapper transform_wrapper_;
        // The subscriber for /accel/filtered
        const ros::Subscriber accel_subscriber_;

        // Queue of received accel messages
        //
        // This will always (after waitUntilReady is called) contain at least one
        // acceleration older than the last update time
        std::vector<geometry_msgs::AccelWithCovarianceStamped> accel_msg_queue_;

        // Subscriber for motor battery voltage
        const ros::Subscriber battery_subscriber_;

        // Queue of received battery messages
        //
        // This will always (after waitUntilReady is called) contain at least one
        // battery message older than the last update time
        std::vector<iarc7_msgs::Float64Stamped> battery_msg_queue_;

        /// Max time to allow for outdated battery messages
        const ros::Duration battery_timeout_;

        // The subscriber for /odometry/filtered
        const ros::Subscriber odometry_subscriber_;

        // Queue of received odometry messages
        //
        // This will always (after waitUntilReady is called) contain at least one
        // velocity older than the last update time
        std::vector<nav_msgs::Odometry> odometry_msg_queue_;

        // The current setpoint
        geometry_msgs::Twist setpoint_;

        // Last time an update was successful
        ros::Time last_update_time_;

        // Max allowed timeout waiting for first velocity and transform
        const ros::Duration startup_timeout_;

        // Max allowed timeout waiting for velocities and transforms
        const ros::Duration update_timeout_;

        // Min allowed requested thrust in m/s^2
        double min_thrust_;

        // Max allowed requested thrust in m/s^2
        double max_thrust_;
    };
}

#endif // QUAD_VELOCITY_CONTROLLER_H

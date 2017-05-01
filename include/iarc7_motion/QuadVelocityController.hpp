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
#include "ros_utils/LinearMsgInterpolator.hpp"
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
        QuadVelocityController(double thrust_pid[6],
                               double pitch_pid[6],
                               double roll_pid[6],
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

        // Use a new thrust model
        void setThrustModel(ThrustModel thrust_model);

        // Require checking of the returned value.
        // Used to update all PID loops according to a time delta that is passed in.
        // Return the uav_command it wants sent to the flight controller.
        bool __attribute__((warn_unused_result)) update(
            const ros::Time& time,
            iarc7_msgs::OrientationThrottleStamped& uav_command);

        /// Waits until this object is ready to begin normal operation
        bool __attribute__((warn_unused_result)) waitUntilReady();

        /// Resets this controller as appropriate for taking over control from another controller
        bool __attribute__((warn_unused_result)) resetForTakeover();

    private:
        /// Looks at setpoint_ and sets our pid controller setpoints accordinly
        /// based on our current yaw
        void updatePidSetpoints(double current_yaw);

        double yawFromQuaternion(const geometry_msgs::Quaternion& rotation);

        // The three PID controllers
        PidController throttle_pid_;
        PidController pitch_pid_;
        PidController roll_pid_;

        ThrustModel thrust_model_;

        ros_utils::SafeTransformWrapper transform_wrapper_;

        // The current setpoint
        geometry_msgs::Twist setpoint_;

        // Last time an update was successful
        ros::Time last_update_time_;

        // Max allowed timeout waiting for first velocity and transform
        const ros::Duration startup_timeout_;

        // Max allowed timeout waiting for velocities and transforms
        const ros::Duration update_timeout_;

        ros_utils::LinearMsgInterpolator<
            geometry_msgs::AccelWithCovarianceStamped,
            tf2::Vector3>
                accel_interpolator_;
        ros_utils::LinearMsgInterpolator<
            iarc7_msgs::Float64Stamped,
            double>
                battery_interpolator_;
        ros_utils::LinearMsgInterpolator<
            nav_msgs::Odometry,
            tf2::Vector3>
                odom_interpolator_;

        // Min allowed requested thrust in m/s^2
        double min_thrust_;

        // Max allowed requested thrust in m/s^2
        double max_thrust_;
    };
}

#endif // QUAD_VELOCITY_CONTROLLER_H

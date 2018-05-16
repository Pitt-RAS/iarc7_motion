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

//Bad Header
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#pragma GCC diagnostic ignored "-Wignored-attributes"
#pragma GCC diagnostic ignored "-Wmisleading-indentation"
#include <Eigen/Geometry>
#pragma GCC diagnostic pop
//End Bad Header

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
#include "iarc7_msgs/MotionPointStamped.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Float32.h"

namespace Iarc7Motion
{

class QuadVelocityController
{
public:
    QuadVelocityController() = delete;

    // Require that PID parameters are passed in upon class creation
    QuadVelocityController(double throttle_pid_settings[6],
                           double pitch_pid_settings[6],
                           double roll_pid_settings[6],
                           double& height_p,
                           const ThrustModel& thrust_model,
                           const ThrustModel& thrust_model_side,
                           const ros::Duration& battery_timeout,
                           ros::NodeHandle& nh,
                           ros::NodeHandle& private_nh);

    ~QuadVelocityController() = default;

    // Don't allow the copy constructor or assignment.
    QuadVelocityController(const QuadVelocityController& rhs) = delete;
    QuadVelocityController& operator=(const QuadVelocityController& rhs) = delete;

    // Set a target velocity for the PID loops
    void setTargetVelocity(iarc7_msgs::MotionPointStamped motion_point);

    // Use a new thrust model
    void setThrustModel(const ThrustModel& thrust_model);

    // Require checking of the returned value.
    // Used to update all PID loops according to a time delta that is passed in.
    // Return the uav_command it wants sent to the flight controller.
    bool __attribute__((warn_unused_result)) update(
        const ros::Time& time,
        iarc7_msgs::OrientationThrottleStamped& uav_command,
        bool z_only=false,
        double pitch=0,
        double roll=0);

    /// Waits until this object is ready to begin normal operation
    bool __attribute__((warn_unused_result)) waitUntilReady();

    /// Prepares this controller as appropriate for taking over control from another controller
    bool __attribute__((warn_unused_result)) prepareForTakeover();

private:
    /// Looks at setpoint_ and sets our pid controller setpoints accordinly
    /// based on our current yaw
    void updatePidSetpoints(double current_yaw, Eigen::VectorXd& odometry);

    double yawFromQuaternion(const geometry_msgs::Quaternion& rotation);

    // The three PID controllers
    PidController throttle_pid_;
    PidController pitch_pid_;
    PidController roll_pid_;

    ThrustModel thrust_model_;
    ThrustModel thrust_model_front_;
    ThrustModel thrust_model_back_;
    ThrustModel thrust_model_left_;
    ThrustModel thrust_model_right_;

    ros_utils::SafeTransformWrapper transform_wrapper_;

    // The current setpoint
    iarc7_msgs::MotionPointStamped setpoint_;

    // The XY plan mixer to use
    std::string xy_mixer_;

    // P term for the position control
    double& height_p_;

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
        Eigen::VectorXd>
            odom_interpolator_;

    // Min allowed requested thrust in m/s^2
    double min_thrust_;

    // Max allowed requested thrust in m/s^2
    double max_thrust_;

    // Min allowed side thrust in m/s^2
    double min_side_thrust_;

    // Max allowed side thrust m/s^2
    double max_side_thrust_;

    // Height below which the drone is required to remain level
    const double level_flight_required_height_;

    // Distance added to the level flight height to trigger
    // entering non-level mode after level flight mode was triggered
    const double level_flight_required_hysteresis_;

    // Flag for whether or not level flight is active
    bool level_flight_active_;
};

}

#endif // QUAD_VELOCITY_CONTROLLER_H

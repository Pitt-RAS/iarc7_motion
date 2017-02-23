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
                               double yaw_pid[5],
                               double hover_throttle,
                               double expected_update_frequency);

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

        /// Waits for the transform to come in at the requested time, returns true if velocities
        /// are valid.
        ///
        /// This must receive two transforms within the timeout period to
        /// consider the velocity valid.
        bool __attribute__((warn_unused_result)) getVelocityAtTime(
            geometry_msgs::Twist& return_velocities,
            const ros::Time& time);

        /// Returns a twist representing the velocity extrapolated from
        /// transform1 and transform2
        static geometry_msgs::Twist twistFromTransforms(
                const geometry_msgs::TransformStamped& transform1,
                const geometry_msgs::TransformStamped& transform2);

        /// Returns the change in yaw between rotation1 and rotation2
        ///
        /// Result is always in the interval (-pi, pi]
        static double yawChangeBetweenOrientations(
                const geometry_msgs::Quaternion& rotation1,
                const geometry_msgs::Quaternion& rotation2);

        /// Returns the yaw represented by rotation (in the interval [-pi, pi])
        static double yawFromQuaternion(
                const geometry_msgs::Quaternion& rotation);

        // Used to store the transform buffer, required for tf2
        tf2_ros::Buffer tfBuffer_;
        // Used to store the transform listener, required for tf2
        tf2_ros::TransformListener tfListener_;

        // The four PID controllers
        PidController throttle_pid_;
        PidController pitch_pid_;
        PidController roll_pid_;
        PidController yaw_pid_;

        // Holds the last transform received to calculate velocities
        geometry_msgs::TransformStamped last_transform_stamped_;

        // Makes sure that we have a lastTransformStamped before returning a
        // valid velocity
        bool have_last_transform_;

        // A fudge feed forward value used for more stable hovering
        double hover_throttle_;

        // The expected frequency at which we update the control loop
        const double expected_update_frequency_;

        static constexpr double INITIAL_TRANSFORM_WAIT_SECONDS{10.0};
        static constexpr double MAX_TRANSFORM_WAIT_SECONDS{1.0};
        static constexpr double MAX_TRANSFORM_DIFFERENCE_SECONDS{0.3};
    };

}

#endif // QUAD_VELOCITY_CONTROLLER_H

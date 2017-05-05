////////////////////////////////////////////////////////////////////////////
//
// Takeoff Controller
//
// Handles takeoff
//
////////////////////////////////////////////////////////////////////////////

#ifndef TAKEOFF_CONTROLLER_H
#define TAKEOFF_CONTROLLER_H

#include <ros/ros.h>

#include "ros_utils/LinearMsgInterpolator.hpp"
#include "ros_utils/SafeTransformWrapper.hpp"

#include "iarc7_motion/ThrustModel.hpp"

// ROS message headers
#include "iarc7_msgs/OrientationThrottleStamped.h"

namespace Iarc7Motion
{

    enum class TakeoffState { RAMP,
                              ASCEND,
                              DONE };

    class TakeoffController
    {
    public:
        TakeoffController() = delete;

        // Require construction with a node handle
        TakeoffController(ros::NodeHandle& nh,
                          ros::NodeHandle& private_nh,
                          const ThrustModel& thrust_model);

        ~TakeoffController() = default;

        // Don't allow the copy constructor or assignment.
        TakeoffController(const TakeoffController& rhs) = delete;
        TakeoffController& operator=(const TakeoffController& rhs) = delete;

        // Used to prepare and check initial conditions for takeoff
        bool __attribute__((warn_unused_result)) prepareForTakeover(
            const ros::Time& time);

        // Used to get a uav control message
        bool __attribute__((warn_unused_result)) update(
            const ros::Time& time,
            iarc7_msgs::OrientationThrottleStamped& uav_command);

        /// Waits until this object is ready to begin normal operation
        bool __attribute__((warn_unused_result)) waitUntilReady();

        bool isDone();

        ThrustModel getThrustModel();

    private:
        ros_utils::SafeTransformWrapper transform_wrapper_;

        TakeoffState state_;

        // Current throttle setting
        double throttle_;

        // Used to hold currently desired thrust model
        ThrustModel thrust_model_;

        // Max allowed takeoff start height
        const double max_takeoff_start_height_;

        const double takeoff_complete_height_;

        const double takeoff_throttle_ramp_rate_;

        const double takeoff_throttle_bump_;

        // Last time an update was successful
        ros::Time last_update_time_;

        // Max allowed timeout waiting for first velocity and transform
        const ros::Duration startup_timeout_;

        // Max allowed timeout waiting for velocities and transforms
        const ros::Duration update_timeout_;
    };

} // End namespace Iarc7Motion

#endif // TAKEOFF_CONTROLLER_H

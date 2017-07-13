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
#include "iarc7_msgs/Float64Stamped.h"
#include "iarc7_msgs/LandingGearContactsStamped.h"
#include "iarc7_msgs/OrientationThrottleStamped.h"

namespace Iarc7Motion
{

enum class TakeoffState { RAMP,
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

    bool __attribute__((warn_unused_result)) calibrateThrustModel(
        const ros::Time& time);

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

    const ThrustModel& getThrustModel() const;

private:
    // Handles incoming landing gear messages
    void processLandingGearMessage(
        const iarc7_msgs::LandingGearContactsStamped::ConstPtr& message);

    static bool allPressed(const iarc7_msgs::LandingGearContactsStamped& msg);

    iarc7_msgs::LandingGearContactsStamped landing_gear_message_;

    ros::Subscriber landing_gear_subscriber_;

    bool landing_gear_message_received_;

    ros_utils::SafeTransformWrapper transform_wrapper_;

    TakeoffState state_;

    // Current throttle setting
    double throttle_;

    // Used to hold currently desired thrust model
    ThrustModel thrust_model_;

    const double takeoff_throttle_ramp_rate_;

    // Last time an update was successful
    ros::Time last_update_time_;

    // Max allowed timeout waiting for first velocity and transform
    const ros::Duration startup_timeout_;

    // Max allowed timeout waiting for velocities and transforms
    const ros::Duration update_timeout_;

    // Max allowed distance between battery messages
    const ros::Duration battery_timeout_;

    // Interpolator for battery voltage
    ros_utils::LinearMsgInterpolator<iarc7_msgs::Float64Stamped, double>
            battery_interpolator_;

    //height from the short ranger LIDAR to the point at which the foot switches should be open
    const double switch_toggle_height_;
};

} // End namespace Iarc7Motion

#endif // TAKEOFF_CONTROLLER_H

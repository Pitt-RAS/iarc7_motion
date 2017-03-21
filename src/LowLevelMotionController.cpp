////////////////////////////////////////////////////////////////////////////
//
// LowLevelMotionController
//
// This is the top level class for the velocity controller.
// It uses an AccelerationPlanner to get velocities.
// Sends them to a QuadVelocityController who runs the PID loops that set the angles
// to output to the flight controller.
// The angles and throttle values are sent to a TwistLimiter class which limits the
// min, max, and max rate of change of those values.
// Finally the processed angles and throttle values are sent out onto a topic.
//
////////////////////////////////////////////////////////////////////////////

#include <ros/ros.h>

#include "iarc7_motion/AccelerationPlanner.hpp"
#include "iarc7_motion/QuadVelocityController.hpp"
#include "iarc7_motion/QuadTwistRequestLimiter.hpp"

#include "iarc7_safety/SafetyClient.hpp"

#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TwistStamped.h"

using namespace Iarc7Motion;
using geometry_msgs::TwistStamped;
using geometry_msgs::Twist;

// This is a helper function that will limit a
// iarc7_msgs::OrientationThrottleStamped using the twist limiter
//
// The twist limiter uses TwistStamped messages to do its work so this
// function converts between the data types.
void limitUavCommand(QuadTwistRequestLimiter& limiter,
                     iarc7_msgs::OrientationThrottleStamped& uav_command)
{
    TwistStamped uav_twist_stamped;
    Twist& uav_twist = uav_twist_stamped.twist;

    // Convert from uav command to twist
    uav_twist_stamped.header.stamp = uav_command.header.stamp;
    uav_twist.linear.z  = uav_command.throttle;
    uav_twist.angular.y = uav_command.data.pitch;
    uav_twist.angular.x = uav_command.data.roll;
    uav_twist.angular.z = uav_command.data.yaw;

    limiter.limitTwist(uav_twist_stamped);

    // Copy the twist to the uav command
    uav_command.header.stamp = uav_twist_stamped.header.stamp;
    uav_command.throttle     = uav_twist.linear.z;
    uav_command.data.pitch   = uav_twist.angular.y;
    uav_command.data.roll    = uav_twist.angular.x;
    uav_command.data.yaw     = uav_twist.angular.z;
}

// Main entry point for the low level motion controller
int main(int argc, char **argv)
{
    // Required by ROS before calling many functions
    ros::init(argc, argv, "Low_Level_Motion_Control");

    ROS_INFO("Low_Level_Motion_Control begin");

    // Create a node handle for the node
    ros::NodeHandle nh;
    // This node handle has a specific namespace that allows us to easily
    // encapsulate parameters
    ros::NodeHandle private_nh ("~");

    // LOAD PARAMETERS
    double throttle_pid[5];
    double pitch_pid[5];
    double roll_pid[5];
    double yaw_pid[5];
    double hover_throttle;
    ThrustModel thrust_model;
    Twist min_velocity, max_velocity, max_velocity_slew_rate;
    double update_frequency;

    // Throttle PID settings retrieve
    private_nh.param("throttle_p", throttle_pid[0], 0.0);
    private_nh.param("throttle_i", throttle_pid[1], 0.0);
    private_nh.param("throttle_d", throttle_pid[2], 0.0);
    private_nh.param("throttle_accumulator_max", throttle_pid[3], 0.0);
    private_nh.param("throttle_accumulator_min", throttle_pid[4], 0.0);

    // Pitch PID settings retrieve
    private_nh.param("pitch_p", pitch_pid[0], 0.0);
    private_nh.param("pitch_i", pitch_pid[1], 0.0);
    private_nh.param("pitch_d", pitch_pid[2], 0.0);
    private_nh.param("pitch_accumulator_max", pitch_pid[3], 0.0);
    private_nh.param("pitch_accumulator_min", pitch_pid[4], 0.0);

    // Roll PID settings retrieve
    private_nh.param("roll_p", roll_pid[0], 0.0);
    private_nh.param("roll_i", roll_pid[1], 0.0);
    private_nh.param("roll_d", roll_pid[2], 0.0);
    private_nh.param("roll_accumulator_max", roll_pid[3], 0.0);
    private_nh.param("roll_accumulator_min", roll_pid[4], 0.0);

    // Yaw PID settings retrieve
    private_nh.param("yaw_p", yaw_pid[0], 0.0);
    private_nh.param("yaw_i", yaw_pid[1], 0.0);
    private_nh.param("yaw_d", yaw_pid[2], 0.0);
    private_nh.param("yaw_accumulator_max", yaw_pid[3], 0.0);
    private_nh.param("yaw_accumulator_min", yaw_pid[4], 0.0);

    // Throttle setting for hovering, to be added to throttle ouput
    private_nh.param("hover_throttle", hover_throttle, 0.0);

    // Thrust model settings retrieve
    private_nh.param("thrust_model/quadcopter_mass",
                     thrust_model.quadcopter_mass,
                     0.0);
    private_nh.param("thrust_model/A_ge", thrust_model.A_ge, 0.0);
    private_nh.param("thrust_model/d0", thrust_model.d0, 0.0);
    std::vector<double> voltage_polynomial;
    private_nh.param("thrust_model/voltage_polynomial", voltage_polynomial);
    ROS_ASSERT_MSG(voltage_polynomial.size()
                == ThrustModel::VOLTAGE_POLYNOMIAL_DEGREE,
                   "Voltage polynomial param is wrong length");
    private_nh.param("thrust_model/throttle_b", thrust_model.throttle_b, 0.0);
    private_nh.param("thrust_model/throttle_c", thrust_model.throttle_c, 0.0);

    // Throttle Limit settings retrieve
    private_nh.param("throttle_max", max_velocity.linear.z, 0.0);
    private_nh.param("throttle_min", min_velocity.linear.z, 0.0);
    private_nh.param("throttle_max_rate", max_velocity_slew_rate.linear.z, 0.0);

    // Pitch Limit settings retrieve
    private_nh.param("pitch_max", max_velocity.angular.y, 0.0);
    private_nh.param("pitch_min", min_velocity.angular.y, 0.0);
    private_nh.param("pitch_max_rate", max_velocity_slew_rate.angular.y, 0.0);

    // Roll Limit settings retrieve
    private_nh.param("roll_max", max_velocity.angular.x, 0.0);
    private_nh.param("roll_min", min_velocity.angular.x, 0.0);
    private_nh.param("roll_max_rate", max_velocity_slew_rate.angular.x, 0.0);

    // Yaw Limit settings retrieve
    private_nh.param("yaw_max", max_velocity.angular.z, 0.0);
    private_nh.param("yaw_min", min_velocity.angular.z, 0.0);
    private_nh.param("yaw_max_rate", max_velocity_slew_rate.angular.z, 0.0);

    // Update frequency retrieve
    private_nh.param("update_frequency", update_frequency, 60.0);

    // Wait for a valid time in case we are using simulated time (not wall time)
    while (ros::ok() && ros::Time::now() == ros::Time(0)) {
        // wait
        ros::spinOnce();
    }

    // Create a quad velocity controller. It will output angles corresponding
    // to our desired velocity
    QuadVelocityController quadController(throttle_pid,
                                          pitch_pid,
                                          roll_pid,
                                          hover_throttle,
                                          thrust_model,
                                          nh,
                                          private_nh);
    if (!quadController.waitUntilReady())
    {
        ROS_ERROR("Failed during initialization of QuadVelocityController");
        return 1;
    }

    // Create an acceleration planner. It handles interpolation between
    // timestamped velocity requests so that smooth accelerations are possible.
    AccelerationPlanner accelerationPlanner(nh);

    // Create the publisher to send the processed uav_commands out with
    // (angles, throttle)
    ros::Publisher uav_control_
        = nh.advertise<iarc7_msgs::OrientationThrottleStamped>(
                "uav_direction_command", 50);

    // Create the publisher to send the current intended velocity target
    ros::Publisher uav_velocity_target_
        = nh.advertise<geometry_msgs::TwistStamped>(
                "cmd_vel", 50);

    // Check for empty uav_control_ as per
    // http://wiki.ros.org/roscpp/Overview/Publishers%20and%20Subscribers
    // section 1
    ROS_ASSERT_MSG(uav_control_,
                   "Could not create uav_direction_command publisher");

    // Create the twist limiter, it will limit min value, max value, and max
    // rate of change.
    QuadTwistRequestLimiter limiter(min_velocity,
                                    max_velocity,
                                    max_velocity_slew_rate);

    // Form a connection with the node monitor. If no connection can be made
    // assert because we don't know what's going on with the other nodes.
    ROS_INFO("low_level_motion: Attempting to form safety bond");
    Iarc7Safety::SafetyClient safety_client(nh, "low_level_motion");
    ROS_ASSERT_MSG(safety_client.formBond(),
                   "low_level_motion: Could not form bond with safety client");

    // Cache the time
    ros::Time last_time = ros::Time::now();

    ros::Rate rate (update_frequency);

    // Run until ROS says we need to shutdown
    while (ros::ok())
    {
        // Check the safety client before updating anything
        //
        // If fatal is active the node monitor is telling everyone to shut
        // down immediately
        ROS_ASSERT_MSG(!safety_client.isFatalActive(),
                       "low_level_motion: fatal event from safety");

        // Get the time
        ros::Time current_time = ros::Time::now();

        // Make sure we don't call QuadVelocity controllers update unless we
        // have a new timestamp to give. This can be a problem with simulated
        // time that does not update with high precision.
        if(current_time > last_time)
        {
            last_time = current_time;

            //  This will contain the target twist or velocity that we want to achieve
            geometry_msgs::TwistStamped target_twist;

            // Check for a safety state in which case we should execute our safety response
            if(safety_client.isSafetyActive())
            {
                // This is the safety response
                // Override whatever the Acceleration Planner wants to do and attempt to land
                target_twist.twist.linear.z = -0.2; // Try to descend
            }
            else
            {
                // If nothing is wrong get a target velocity from the acceleration planner
                accelerationPlanner.getTargetTwist(current_time, target_twist);
            }

            // Publish the current target velocity
            uav_velocity_target_.publish(target_twist);

            // Request the appropriate throttle and angle settings for the desired velocity
            quadController.setTargetVelocity(target_twist.twist);

            // Get the next uav command that is appropriate for the desired velocity
            iarc7_msgs::OrientationThrottleStamped uav_command;
            bool success = quadController.update(current_time, uav_command);
            ROS_ASSERT_MSG(success, "LowLevelMotion controller update failed");

            // Limit the uav command with the twist limiter before sending the uav command
            limitUavCommand(limiter, uav_command);

            // Publish the desired angles and throttle to the topic
            uav_control_.publish(uav_command);
        }

        // Handle all ROS callbacks
        ros::spinOnce();
        rate.sleep();
    }

    // All is good.
    return 0;
}

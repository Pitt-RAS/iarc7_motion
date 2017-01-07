////////////////////////////////////////////////////////////////////////////
//
// Quad Velocity controller
//
// Implements details involving the throttle controller
//
////////////////////////////////////////////////////////////////////////////

#include <sstream>

// Associated header
#include "iarc7_motion/QuadVelocityController.hpp"

// ROS message headers

using namespace Iarc7Motion;

QuadVelocityController::QuadVelocityController(double thrust_pid[5],
                                               double pitch_pid[5],
                                               double roll_pid[5],
                                               double yaw_pid[5]) :
tfBuffer_(),
tfListener_(tfBuffer_),
throttle_pid_(thrust_pid[0], thrust_pid[1], thrust_pid[2], thrust_pid[3], thrust_pid[4]),
pitch_pid_(pitch_pid[0], pitch_pid[1], pitch_pid[2], pitch_pid[3], pitch_pid[4]),
roll_pid_(roll_pid[0], roll_pid[1], roll_pid[2], roll_pid[3], roll_pid[4]),
yaw_pid_(yaw_pid[0], yaw_pid[1], yaw_pid[2], yaw_pid[3], yaw_pid[4]),
hover_throttle_(58.0),
last_transform_stamped_(),
last_yaw_(0.0),
have_last_velocity_stamped_(false),
ran_once_(false)
{

}

// TODO sanity check these values
void QuadVelocityController::setTargetVelocity(geometry_msgs::Twist twist)
{
    throttle_pid_.setSetpoint(twist.linear.z);
    yaw_pid_.setSetpoint(twist.angular.z);

    // Pitch and roll velocities are transformed to vectors that pitch and roll can respond to
    pitch_pid_.setSetpoint(twist.linear.x * std::cos(last_yaw_) + twist.linear.y * std::sin(last_yaw_));

    // Invert roll because a positive y velocity means a negative roll by the right hand rule
    roll_pid_.setSetpoint(-(twist.linear.x * -std::sin(last_yaw_) + twist.linear.y * std::cos(last_yaw_)));
}

// Needs to be called at regular intervals in order to keep catching the latest velocities.
bool QuadVelocityController::update(const ros::Time& time,
                                    iarc7_msgs::OrientationThrottleStamped& uav_command)
{
    geometry_msgs::Twist velocities;
    bool success = getVelocities(velocities);
    if (!success) {
        ROS_WARN("Failed to get current velocities in QuadVelocityController::update");
        return false;
    }

    // Update all the PID loops
    double throttle_output;
    double pitch_output;
    double roll_output;
    double yaw_output;

    success = throttle_pid_.update(velocities.linear.z, time, throttle_output);
    if (!success) {
        ROS_WARN("Throttle PID update failed in QuadVelocityController::update");
        return false;
    }

    success = pitch_pid_.update(velocities.linear.x, time, pitch_output);
    if (!success) {
        ROS_WARN("Pitch PID update failed in QuadVelocityController::update");
        return false;
    }

    success = roll_pid_.update(-velocities.linear.y, time, roll_output);
    if (!success) {
        ROS_WARN("Roll PID update failed in QuadVelocityController::update");
        return false;
    }

    success = yaw_pid_.update(velocities.angular.z, time, yaw_output);
    if (!success) {
        ROS_WARN("Yaw PID update failed in QuadVelocityController::update");
        return false;
    }

    uav_command.header.stamp = time;
    uav_command.throttle = throttle_output + hover_throttle_;
    uav_command.data.pitch = pitch_output;
    uav_command.data.roll = roll_output;
    uav_command.data.yaw = yaw_output;

    if (!std::isfinite(uav_command.throttle)
     || !std::isfinite(uav_command.data.pitch)
     || !std::isfinite(uav_command.data.roll)
     || !std::isfinite(uav_command.data.yaw)) {
        ROS_WARN(
            "Part of command is not finite in QuadVelocityController::update");
        return false;
    }

    // Print the velocity and throttle information
    ROS_INFO("Vz:       %f Vx:    %f Vy:   %f Vyaw: %f", velocities.linear.z, velocities.linear.x, velocities.linear.y, velocities.angular.z);
    ROS_INFO("Throttle: %f Pitch: %f Roll: %f Yaw:  %f", uav_command.throttle, uav_command.data.pitch, uav_command.data.roll, uav_command.data.yaw);

    return true;
}

bool QuadVelocityController::getTransformAfterTime(
        const ros::Time& time,
        geometry_msgs::TransformStamped& transform,
        const ros::Time& latest_time_allowed) {

    try
    {
        while (ros::ok())
        {
            if (ros::Time::now() > time + ros::Duration(MAX_TRANSFORM_WAIT_SECONDS))
            {
                ROS_ERROR_STREAM("Transform timed out ("
                                 << "current time: " << ros::Time::now() << ", "
                                 << "request time: " << time
                                 << ")");
                return false;
            }
            else if (tfBuffer_.canTransform("map", "quad", time))
            {
                transform = tfBuffer_.lookupTransform("map", "quad", time);

                if (latest_time_allowed != ros::Time(0)
                    && transform.header.stamp > latest_time_allowed)
                {
                    ROS_ERROR_STREAM(
                        "First available transform came after latest allowed time"
                        << " (transform time: " << transform.header.stamp
                        << ", latest allowed time: " << latest_time_allowed
                        << ")");
                    return false;
                }
                else
                {
                    return true;
                }
            }
            else
            {
                // Check if we can transform at the current time, but not
                // the original time, which means the requested time was
                // too old.  If so, we accept this newer transform instead.
                if (tfBuffer_.canTransform("map", "quad", ros::Time(0)))
                {
                    transform = tfBuffer_.lookupTransform("map", "quad", ros::Time(0));

                    if (latest_time_allowed != ros::Time(0)
                        && transform.header.stamp > latest_time_allowed)
                    {
                        ROS_ERROR_STREAM(
                            "First available transform came after latest allowed time"
                            << " (transform time: " << transform.header.stamp
                            << ", latest allowed time: " << latest_time_allowed
                            << ")");
                        return false;
                    }

                    if (transform.header.stamp > time)
                    {
                        return true;
                    }
                }

                ros::spinOnce();
                ros::Duration(0.005).sleep();
            }
        }
    }
    catch (tf2::TransformException& ex)
    {
        ROS_ERROR("Could not transform map to level_quad: %s",ex.what());
    }

    ROS_ERROR("ros::ok false while waiting for transform");
    return false;
}

bool QuadVelocityController::getVelocities(geometry_msgs::Twist& return_velocities)
{
    // Get the map to level quad transform
    geometry_msgs::TransformStamped transformStamped;

    // Get current ROS time
    ros::Time time = ros::Time::now();

    // Check if we already have a velocity at this time
    if (have_last_velocity_stamped_ && time == last_velocity_stamped_.header.stamp) {
        return_velocities = last_velocity_stamped_.twist;
        return true;
    }

    // We don't have a velocity at this time yet, so make sure we don't fetch
    // a transform at the same time as the last one
    while (ros::ok()
            && ran_once_
            && time == last_transform_stamped_.header.stamp) {
        ros::spinOnce();
    }

    // Fetch the new transform
    ros::Time latest_time_allowed = ros::Time(0);
    if (ran_once_)
    {
        latest_time_allowed = last_transform_stamped_.header.stamp
                            + ros::Duration(MAX_TRANSFORM_DIFFERENCE_SECONDS);
    }
    bool fetched_transform = getTransformAfterTime(time,
                                                   transformStamped,
                                                   latest_time_allowed);
    if (!fetched_transform)
    {
        ROS_ERROR("Failed to fetch new transform in QuadVelocityController");
        ran_once_ = false;
        return false;
    }

    // Get the transforms without the stamps for readability
    geometry_msgs::Transform& transform = transformStamped.transform;
    geometry_msgs::Transform& oldTransform = last_transform_stamped_.transform;

    // Get the yaw (z axis) rotation from the quanternion
    double ysqr = transform.rotation.y * transform.rotation.y;
    double t3 = 2.0f * (transform.rotation.w * transform.rotation.z + transform.rotation.x * transform.rotation.y);
    double t4 = 1.0f - 2.0f * (ysqr + transform.rotation.z * transform.rotation.z);  
    double current_yaw = std::atan2(t3, t4);

    if(!ran_once_)
    {
        ran_once_ = true;
        last_transform_stamped_ = transformStamped;
        last_yaw_ = current_yaw;
        return getVelocities(return_velocities);
    }

    // Get the time between the two transforms
    ros::Duration delta_seconds = transformStamped.header.stamp - last_transform_stamped_.header.stamp;

    // Calculate x, y, and z velocity
    // X and Y are transformed using the current yaw heading to velocities that the pitch and roll can respond to directly
    double delta = delta_seconds.toSec();
    double world_x_vel = ((transform.translation.x - oldTransform.translation.x) / delta);
    double world_y_vel = ((transform.translation.y - oldTransform.translation.y) / delta);
    return_velocities.linear.x = world_x_vel * std::cos(current_yaw) + world_y_vel * std::sin(current_yaw);
    return_velocities.linear.y = world_x_vel * -std::sin(current_yaw) + world_y_vel * std::cos(current_yaw);

    return_velocities.linear.z = (transform.translation.z - oldTransform.translation.z) / delta;

    double yaw_difference = current_yaw - last_yaw_;

    // The next two if statements handle if the yaw_difference is large due to gimbal lock
    // Assumes that we don't have a yaw_difference more than 2pi. We won't since we get all the angles
    // from an atan2 which only return -pi to pi
    // Also assumes that we won't have a rotation difference more than pi
    if(yaw_difference > M_PI)
    {
        yaw_difference = yaw_difference - 2 * M_PI;
    }

    if(yaw_difference < -M_PI)
    {
        yaw_difference = yaw_difference + 2 * M_PI;
    }

    return_velocities.angular.z = yaw_difference / delta;

    // Store this as the last valid velocity
    have_last_velocity_stamped_ = true;
    last_velocity_stamped_.twist = return_velocities;
    last_velocity_stamped_.header.stamp = transformStamped.header.stamp;

    last_transform_stamped_ = transformStamped;
    last_yaw_ = current_yaw;

    return true;
}

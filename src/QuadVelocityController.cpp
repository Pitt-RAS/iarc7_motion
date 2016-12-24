////////////////////////////////////////////////////////////////////////////
//
// Quad Velocity controller
//
// Implements details involving the throttle controller
//
////////////////////////////////////////////////////////////////////////////

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
    getVelocities(velocities);

    // Update all the PID loops
    double throttle_output;
    double pitch_output;
    double roll_output;
    double yaw_output;

    bool success = throttle_pid_.update(velocities.linear.z, time, throttle_output);
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

    // For now publish, should send values to a hard limiter first
    uav_command.header.stamp = time;
    uav_command.throttle = throttle_output + hover_throttle_;
    uav_command.data.pitch = pitch_output;
    uav_command.data.roll = roll_output;
    uav_command.data.yaw = yaw_output;

    // Print the velocity and throttle information
    ROS_INFO("Vz:       %f Vx:    %f Vy:   %f Vyaw: %f", velocities.linear.z, velocities.linear.x, velocities.linear.y, velocities.angular.z);
    ROS_INFO("Throttle: %f Pitch: %f Roll: %f Yaw:  %f", uav_command.throttle, uav_command.data.pitch, uav_command.data.roll, uav_command.data.yaw);

    return true;
}

// Waits for the next transform to come in, returns true if velocities are valid
// Has to receive two transforms within the timeout period to consider the velocity valid
bool QuadVelocityController::getVelocities(geometry_msgs::Twist& return_velocities)
{
    // Can be set to mark a velocity reading invalid
    bool velocities_valid{true};

    // Get the map to level quad transform
    geometry_msgs::TransformStamped transformStamped;

    // Get current ROS time
    ros::Time time = ros::Time::now();

    // Check if we already have a velocity at this time
    if (time == last_velocity_stamped_.header.stamp) {
        return_velocities = last_velocity_stamped_.twist;
        return true;
    }

    try{
        transformStamped = tfBuffer_.lookupTransform("map", "quad", time, ros::Duration(MAX_TRANSFORM_WAIT_SECONDS));
        double current_yaw{0.0};
        if(ran_once_)
        {
            // Get the time between the two transforms
            ros::Duration delta_seconds = transformStamped.header.stamp - last_transform_stamped_.header.stamp;

            if(delta_seconds > ros::Duration(MAX_TRANSFORM_DIFFERENCE_SECONDS))
            {
                ROS_ERROR("Velocities invalid, time between transforms is too high: %f seconds", delta_seconds.toSec());
                velocities_valid = false;
            }

            // Get the transforms without the stamps for readability
            geometry_msgs::Transform& transform = transformStamped.transform;
            geometry_msgs::Transform& oldTransform = last_transform_stamped_.transform;

            // Get the yaw (z axis) rotation from the quanternion
            double ysqr = transform.rotation.y * transform.rotation.y;
            double t3 = 2.0f * (transform.rotation.w * transform.rotation.z + transform.rotation.x * transform.rotation.y);
            double t4 = 1.0f - 2.0f * (ysqr + transform.rotation.z * transform.rotation.z);  
            current_yaw = std::atan2(t3, t4);

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
            last_velocity_stamped_.twist = return_velocities;
            last_velocity_stamped_.header.stamp = time;
        }
        else
        {
            velocities_valid = false;
            ran_once_ = true;
        }

        last_transform_stamped_ = transformStamped;
        last_yaw_ = current_yaw;
    }
    catch (tf2::TransformException& ex){
        ROS_ERROR("Could not transform map to level_quad: %s",ex.what());
        velocities_valid = false;
        ran_once_ = false;
    }

    return velocities_valid;
}

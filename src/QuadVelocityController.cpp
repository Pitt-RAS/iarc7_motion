////////////////////////////////////////////////////////////////////////////
//
// Quad Velocity controller
//
// Accepts a target velocity and uses 4 PID loops (pitch, yaw, roll, thrust)
// To attempt to hit the target velocity.
//
////////////////////////////////////////////////////////////////////////////

#include <cmath>
#include <boost/algorithm/clamp.hpp>

// Associated header
#include "iarc7_motion/QuadVelocityController.hpp"

// ROS Headers
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

// ROS message headers
#include "geometry_msgs/AccelWithCovarianceStamped.h"
#include "iarc7_msgs/Float64Stamped.h"

using namespace Iarc7Motion;

template<typename T>
static T getParam(ros::NodeHandle& nh, const std::string& name)
{
    T val;
    ROS_ASSERT_MSG(nh.getParam(name, val),
                   "Failed to retrieve parameter: %s",
                   name.c_str());
    return val;
}

// Create QuadVelocityController, copy PID settings, initialize all other variables
QuadVelocityController::QuadVelocityController(double thrust_pid[6],
                                               double pitch_pid[6],
                                               double roll_pid[6],
                                               const ThrustModel& thrust_model,
                                               const ros::Duration& battery_timeout,
                                               ros::NodeHandle& nh,
                                               ros::NodeHandle& private_nh) :
throttle_pid_(thrust_pid[0], thrust_pid[1], thrust_pid[2], thrust_pid[3], thrust_pid[4], thrust_pid[5]),
pitch_pid_(pitch_pid[0], pitch_pid[1], pitch_pid[2], pitch_pid[3], pitch_pid[4], pitch_pid[5]),
roll_pid_(roll_pid[0], roll_pid[1], roll_pid[2], roll_pid[3], roll_pid[4], roll_pid[5]),
thrust_model_(thrust_model),
tfBuffer_(),
tfListener_(tfBuffer_),
accel_subscriber_(nh.subscribe(
    "accel/filtered",
    100,
    boost::function<void(const geometry_msgs::AccelWithCovarianceStamped::ConstPtr&)>(
        boost::bind(&QuadVelocityController::msgCallback
                        <geometry_msgs::AccelWithCovarianceStamped>,
                    this,
                    _1,
                    boost::ref(accel_msg_queue_))))),
accel_msg_queue_(),
battery_subscriber_(nh.subscribe(
    "fc_battery",
    100,
    boost::function<void(const iarc7_msgs::Float64Stamped::ConstPtr&)>(
        boost::bind(&QuadVelocityController::msgCallback
                        <iarc7_msgs::Float64Stamped>,
                    this,
                    _1,
                    boost::ref(battery_msg_queue_))))),
battery_msg_queue_(),
battery_timeout_(battery_timeout),
odometry_subscriber_(nh.subscribe(
    "odometry/filtered",
    100,
    boost::function<void(const nav_msgs::Odometry::ConstPtr&)>(
        boost::bind(&QuadVelocityController::msgCallback<nav_msgs::Odometry>,
                    this,
                    _1,
                    boost::ref(odometry_msg_queue_))))),
odometry_msg_queue_(),
setpoint_(),
startup_timeout_(getParam<double>(private_nh, "startup_timeout")),
update_timeout_(getParam<double>(private_nh, "update_timeout")),
min_thrust_(getParam<double>(private_nh, "min_thrust")),
max_thrust_(getParam<double>(private_nh, "max_thrust"))
{
}

// Take in a target velocity that does not take into account the quads current heading
// And transform it to the velocity vectors that correspond to the quads current yaw
// Set the PID's set points accordingly
void QuadVelocityController::setTargetVelocity(geometry_msgs::Twist twist)
{
    setpoint_ = twist;
}

// Main update, runs all PID calculations and returns a desired uav_command
// Needs to be called at regular intervals in order to keep catching the latest velocities.
bool QuadVelocityController::update(const ros::Time& time,
                                    iarc7_msgs::OrientationThrottleStamped& uav_command)
{
    if (time < last_update_time_) {
        ROS_ERROR("Tried to update QuadVelocityController with time before last update");
        return false;
    }

    // Get the current velocity of the quad.
    tf2::Vector3 velocity;
    bool success = getInterpolatedMsgAtTime<nav_msgs::Odometry, tf2::Vector3>(
            odometry_msg_queue_,
            velocity,
            time,
            ros::Duration(0),
            update_timeout_,
            [](const nav_msgs::Odometry& msg) {
                return tf2::Vector3(msg.twist.twist.linear.x,
                                    msg.twist.twist.linear.y,
                                    msg.twist.twist.linear.z);
            });
    if (!success) {
        ROS_WARN("Failed to get current velocities in QuadVelocityController::update");
        return false;
    }

    // Get the current battery voltage of the quad
    double voltage;
    success = getInterpolatedMsgAtTime<iarc7_msgs::Float64Stamped, double>(
            battery_msg_queue_,
            voltage,
            time,
            battery_timeout_,
            update_timeout_,
            [](const iarc7_msgs::Float64Stamped& msg) {
                return msg.data;
            });
    if (!success) {
        ROS_WARN("Failed to get current battery voltage in QuadVelocityController::update");
        return false;
    }

    // Get the current acceleration of the quad
    tf2::Vector3 accel;
    success = getInterpolatedMsgAtTime
                    <geometry_msgs::AccelWithCovarianceStamped, tf2::Vector3>(
            accel_msg_queue_,
            accel,
            time,
            ros::Duration(0),
            update_timeout_,
            [](const geometry_msgs::AccelWithCovarianceStamped& msg) {
                return tf2::Vector3(msg.accel.accel.linear.x,
                                    msg.accel.accel.linear.y,
                                    msg.accel.accel.linear.z);
            });
    if (!success) {
        ROS_WARN("Failed to get current acceleration in QuadVelocityController::update");
        return false;
    }


    // Get the current transform (rotation) of the quad
    geometry_msgs::TransformStamped transform;
    success = getTransformAtTime(transform,
                                 "quad",
                                 "level_quad",
                                 time,
                                 update_timeout_);
    if (!success) {
        ROS_WARN("Failed to get current transform in QuadVelocityController::update");
        return false;
    }

    // Get the current transform (rotation) of the quad
    geometry_msgs::TransformStamped col_height_transform;
    success = getTransformAtTime(col_height_transform,
                                 "center_of_lift",
                                 "map",
                                 time,
                                 update_timeout_);
    if (!success) {
        ROS_WARN("Failed to get current transform in QuadVelocityController::update");
        return false;
    }
    double col_height = col_height_transform.transform.translation.z;

    // Get current yaw from the transform
    double current_yaw = yawFromQuaternion(transform.transform.rotation);

    // Update setpoints on PID controllers
    updatePidSetpoints(current_yaw);

    // Update all the PID loops

    // Used to temporarily store throttle and angle outputs from PID loops
    double vertical_accel_output;
    double pitch_output;
    double roll_output;

    // Update throttle PID loop
    success = throttle_pid_.update(velocity.z(),
                                   time,
                                   vertical_accel_output,
                                   accel.z());
    if (!success) {
        ROS_WARN("Throttle PID update failed in QuadVelocityController::update");
        return false;
    }

    // Calculate local frame velocities
    double local_x_velocity = std::cos(current_yaw) * velocity.x()
                            + std::sin(current_yaw) * velocity.y();
    double local_y_velocity = -std::sin(current_yaw) * velocity.x()
                            +  std::cos(current_yaw) * velocity.y();
    // Calculate local frame accelerations
    double local_x_accel = std::cos(current_yaw) * accel.x()
                         + std::sin(current_yaw) * accel.y();
    double local_y_accel = -std::sin(current_yaw) * accel.x()
                         +  std::cos(current_yaw) * accel.y();

    // Update pitch PID loop
    success = pitch_pid_.update(local_x_velocity,
                                time,
                                pitch_output,
                                local_x_accel);
    if (!success) {
        ROS_WARN("Pitch PID update failed in QuadVelocityController::update");
        return false;
    }

    // Update roll PID loop
    success = roll_pid_.update(-local_y_velocity,
                               time,
                               roll_output,
                               -local_y_accel);
    if (!success) {
        ROS_WARN("Roll PID update failed in QuadVelocityController::update");
        return false;
    }

    // Fill in the uav_command's information
    uav_command.header.stamp = time;

    double hover_accel = 9.8;
    //based on roll and pitch angles we calculate additional throttle to match the level hover_accel
    double tilt_accel = hover_accel*(1-cos(roll_output)*cos(pitch_output));


    // Simple feedforward using a fixed hover_accel to avoid excessive
    // oscillations from the PID's I term compensating for there needing to be
    // an average throttle value at 0 velocity in the z axis.
    ROS_DEBUG("Thrust: %f, Voltage: %f, height: %f", hover_accel + tilt_accel + vertical_accel_output, voltage, col_height);
    double thrust_request = hover_accel + tilt_accel + vertical_accel_output;
    uav_command.throttle = thrust_model_.throttleFromAccel(
            boost::algorithm::clamp(thrust_request, min_thrust_, max_thrust_),
            voltage,
            col_height);
    uav_command.data.pitch = pitch_output;
    uav_command.data.roll = roll_output;

    // Yaw rate needs no correction because the input and output are both
    // velocities
    uav_command.data.yaw = -setpoint_.angular.z;

    // Check that the PID loops did not return invalid values before returning
    if (!std::isfinite(uav_command.throttle)
     || !std::isfinite(uav_command.data.pitch)
     || !std::isfinite(uav_command.data.roll)
     || !std::isfinite(uav_command.data.yaw)) {
        ROS_WARN(
            "Part of command is not finite in QuadVelocityController::update");
        return false;
    }

    // Print the velocity and throttle information
    ROS_DEBUG("Vz: %f Vx: %f Vy: %f",
             velocity.z(),
             velocity.x(),
             velocity.y());
    ROS_DEBUG("Throttle: %f Pitch: %f Roll: %f Yaw: %f",
             uav_command.throttle,
             uav_command.data.pitch,
             uav_command.data.roll,
             uav_command.data.yaw);

    last_update_time_ = time;
    return true;
}

bool QuadVelocityController::waitUntilReady()
{
    const ros::Time start_time = ros::Time::now();
    while (ros::ok()
           && accel_msg_queue_.empty()
           && ros::Time::now() < start_time + startup_timeout_) {
        ros::spinOnce();
        ros::Duration(0.005).sleep();
    }

    if (accel_msg_queue_.empty()) {
        ROS_ERROR("Failed to fetch initial acceleration");
        return false;
    }

    while (ros::ok()
           && odometry_msg_queue_.empty()
           && ros::Time::now() < start_time + startup_timeout_) {
        ros::spinOnce();
        ros::Duration(0.005).sleep();
    }

    if (odometry_msg_queue_.empty()) {
        ROS_ERROR("Failed to fetch initial velocity");
        return false;
    }

    while (ros::ok()
           && battery_msg_queue_.empty()
           && ros::Time::now() < start_time + startup_timeout_) {
        ros::spinOnce();
        ros::Duration(0.005).sleep();
    }

    if (battery_msg_queue_.empty()) {
        ROS_ERROR("Failed to fetch battery voltage");
        return false;
    }

    geometry_msgs::TransformStamped transform;
    bool success = getTransformAtTime(transform,
                                      "quad",
                                      "level_quad",
                                      ros::Time(0),
                                      startup_timeout_);
    if (!success)
    {
        ROS_ERROR("Failed to fetch initial transform");
        return false;
    }

    success = getTransformAtTime(transform,
                                 "center_of_lift",
                                 "map",
                                 ros::Time(0),
                                 startup_timeout_);
    if (!success)
    {
        ROS_ERROR("Failed to fetch initial transform");
        return false;
    }

    // Mark the last update time as 1ns later than the odometry message,
    // because we should always have a message older than the last update
    last_update_time_ = odometry_msg_queue_.front().header.stamp
                      + ros::Duration(0, 1);
    if (last_update_time_ <= battery_msg_queue_.front().header.stamp) {
        last_update_time_ = battery_msg_queue_.front().header.stamp
                          + ros::Duration(0, 1);
    }
    return true;
}

// Get a valid transform, blocks until the transform is received or
// the request takes too long and times out
bool QuadVelocityController::getTransformAtTime(
        geometry_msgs::TransformStamped& transform,
        const std::string& target_frame,
        const std::string& source_frame,
        const ros::Time& time,
        const ros::Duration& timeout) const
{
    const ros::Time start_time = ros::Time::now();

    // Catch TransformException exceptions
    try
    {
        // While we aren't supposed to be shutting down
        while (ros::ok())
        {
            if (ros::Time::now() > start_time + timeout)
            {
                ROS_ERROR_STREAM("Transform timed out ("
                              << "current time: " << ros::Time::now() << ", "
                              << "request time: " << start_time
                              << ")");
                return false;
            }

            // Check if the transform from map to quad can be made right now
            if (tfBuffer_.canTransform(source_frame, target_frame, time))
            {
                // Get the transform
                transform = tfBuffer_.lookupTransform(source_frame, target_frame, time);
                return true;
            }

            // Handle callbacks and sleep for a small amount of time
            // before looping again
            ros::spinOnce();
            ros::Duration(0.005).sleep();
        }
    }
    // Catch any exceptions that might happen while transforming
    catch (tf2::TransformException& ex)
    {
        ROS_ERROR("Exception transforming %s to %s: %s",
                  target_frame.c_str(),
                  source_frame.c_str(),
                  ex.what());
    }

    ROS_ERROR("Exception or ros::ok was false");
    return false;
}

template<class StampedMsgType, class OutType>
bool QuadVelocityController::getInterpolatedMsgAtTime(
        typename std::vector<StampedMsgType>& queue,
        OutType& out,
        const ros::Time& time,
        const ros::Duration& allowed_time_offset,
        const ros::Duration& timeout,
        const std::function<OutType(const StampedMsgType&)>& extractor) const
{
    // Wait until there's a message with stamp >= time
    if (!waitForMsgAtTime(queue, time - allowed_time_offset, timeout))
    {
        ROS_ERROR("Timed out waiting for message in getInterpolatedMsgAtTime");
        return false;
    }

    if (queue.back().header.stamp <= time - allowed_time_offset) {
        ROS_ERROR("Haven't received a message within the last %f seconds",
                  allowed_time_offset.toSec());
        return false;
    }

    if (queue.back().header.stamp <= time) {
        out = extractor(queue.back());
        return true;
    }

    // Get first msg that is >= the requested time
    typename std::vector<StampedMsgType>::iterator geq_msg
        = std::lower_bound(queue.begin(),
                           queue.end(),
                           time,
                           &QuadVelocityController::timeVsMsgStampedComparator
                                <StampedMsgType>);

    ROS_ASSERT_MSG(geq_msg > queue.begin(), "geq_msg - 1 out of bounds");
    ROS_ASSERT_MSG(geq_msg < queue.end(), "geq_msg out of bounds");

    // Linear interpolation between next_msg and last_msg
    const StampedMsgType& next_msg = *geq_msg;
    const StampedMsgType& last_msg = *(geq_msg - 1);

    const OutType& next_data = extractor(next_msg);
    const OutType& last_data = extractor(last_msg);

    double dt = (next_msg.header.stamp - last_msg.header.stamp).toSec();

    out = ((time - last_msg.header.stamp).toSec() / dt) * next_data
        + ((next_msg.header.stamp - time).toSec() / dt) * last_data;
    return true;
}

void QuadVelocityController::updatePidSetpoints(double current_yaw)
{
    throttle_pid_.setSetpoint(setpoint_.linear.z);

    // Pitch and roll velocities are transformed according to the last yaw
    // angle because the incoming target velocities are in the map frame
    double local_x_velocity = setpoint_.linear.x * std::cos(current_yaw)
                            + setpoint_.linear.y * std::sin(current_yaw);
    double local_y_velocity = setpoint_.linear.x * -std::sin(current_yaw)
                            + setpoint_.linear.y *  std::cos(current_yaw);

    pitch_pid_.setSetpoint(local_x_velocity);

    // Note: Roll is inverted because a positive y velocity means a negative
    // roll by the right hand rule
    roll_pid_.setSetpoint(-local_y_velocity);
}

template<class StampedMsgType>
bool QuadVelocityController::waitForMsgAtTime(
        const std::vector<StampedMsgType>& queue,
        const ros::Time& time,
        const ros::Duration& timeout) const
{
    if (queue.front().header.stamp >= last_update_time_) {
        ROS_ERROR("Class invariant false: queue does not contain a message older than the requested time");
        return false;
    }

    const ros::Time start_time = ros::Time::now();
    while (ros::ok()
           && queue.back().header.stamp < time
           && ros::Time::now() < start_time + timeout) {
        ros::spinOnce();
        ros::Duration(0.005).sleep();
    }

    return (queue.back().header.stamp >= time);
}

template<class StampedMsgType>
void QuadVelocityController::msgCallback(
        const typename StampedMsgType::ConstPtr& msg,
        std::vector<StampedMsgType>& queue)
{
    if (!queue.empty() && msg->header.stamp < queue.back().header.stamp) {
        ROS_ERROR_STREAM("Ignoring message with timestamp of "
                      << msg->header.stamp
                      << " because the queue already has a message with stamp "
                      << queue.back().header.stamp);
        return;
    }

    queue.push_back(*msg);

    // Keep only one message older than the last update time
    typename std::vector<StampedMsgType>::const_iterator first_geq_time
        = std::lower_bound(queue.begin(),
                           queue.end(),
                           last_update_time_,
                           &QuadVelocityController::timeVsMsgStampedComparator
                                <StampedMsgType>);
    if (first_geq_time - 1 > queue.begin()) {
        queue.erase(queue.begin(), first_geq_time - 1);
    }
}

double QuadVelocityController::yawFromQuaternion(
        const geometry_msgs::Quaternion& rotation)
{
    tf2::Quaternion quaternion;
    tf2::convert(rotation, quaternion);

    tf2::Matrix3x3 matrix;
    matrix.setRotation(quaternion);

    double y, p, r;
    matrix.getEulerYPR(y, p, r);

    return y;
}

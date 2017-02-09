////////////////////////////////////////////////////////////////////////////
//
// Acceleration Planner
//
// Stores a queue of velocity commands from a topic. Interpolates
// between velocities when a velocity is requested. Allows for smooth
// acceleration.
//
////////////////////////////////////////////////////////////////////////////

// Associated header
#include "iarc7_motion/AccelerationPlanner.hpp"

//System Headers
#include <algorithm>

using namespace Iarc7Motion;

// Construct the object need a node handle to register the subscriber for velocity targets
AccelerationPlanner::AccelerationPlanner(ros::NodeHandle& nh) :
velocity_targets_subscriber_(),
velocity_targets_()
{
    velocity_targets_subscriber_ = nh.subscribe("movement_velocity_targets", 100, &AccelerationPlanner::processVelocityCommand, this);

    // Put an initial time in the buffer so that buffer is never empty
    TwistStamped zero_velocity;
    zero_velocity.header.stamp = ros::Time::now();
    velocity_targets_.emplace_back(zero_velocity);
}

// Called by a class user to get a target twist.
// Makes the call to trim the velocity queue according to the current time
// Interpolates if there is more than one twist
void AccelerationPlanner::getTargetTwist(const ros::Time& current_time, geometry_msgs::TwistStamped& target_twist)
{

    // Trim velocity queue when done we will have one or two velocities available.
    trimVelocityQueue(velocity_targets_, current_time);

    if(velocity_targets_.size() == 1U)
    {
        target_twist = velocity_targets_[0];
    }
    else
    {
        bool success = interpolateTwists(velocity_targets_[0], velocity_targets_[1], target_twist, current_time);
        if(!success)
        {
            ROS_ERROR("Interpolation failed, not dispatching velocity");
            return;
        }
    }
}

// Takes two twists and interpolates between their values using linear interpolation
bool AccelerationPlanner::interpolateTwists(TwistStamped& begin, TwistStamped& end, TwistStamped& interpolated, ros::Time time)
{

    ros::Time begin_time = begin.header.stamp;
    ros::Time end_time   = end.header.stamp;

    // Check for incorrect times
    if(end_time < begin_time)
    {
        return false;
    }
    
    if(time > end_time)
    {
        return false;
    }

    if(time < begin_time)
    {
        return false;
    }

    // Find the ratio between the desired delta time and the delta time between the two twists
    double point_on_line = (time - begin_time).toSec() / (end_time - begin_time).toSec();

    // Interpolate twist using ax+b
    interpolated.twist.linear.x = (end.twist.linear.x - begin.twist.linear.x) * point_on_line + begin.twist.linear.x;
    interpolated.twist.linear.y = (end.twist.linear.y - begin.twist.linear.y) * point_on_line + begin.twist.linear.y;
    interpolated.twist.linear.z = (end.twist.linear.z - begin.twist.linear.z) * point_on_line + begin.twist.linear.z;
    interpolated.twist.angular.z = (end.twist.angular.z - begin.twist.angular.z) * point_on_line + begin.twist.angular.z;

    // Set the current header stamp
    interpolated.header.stamp = time;

    return true;
}

// Time the velocity queue so that we don't have old velocity targets in the queue
bool AccelerationPlanner::trimVelocityQueue(TwistStampedArray& twists, const ros::Time& time)
{
    // Check for empty array of twists
    if(twists.empty())
    {
        ROS_ERROR("trimVelocityQueue detected no velocities available");
        return false;
    }

    // We want to get rid of all timestamps except for the one before the current time and then also
    // keep all going into the future.

    // Find the first time more than or equal
    // Uses a lambda to compare the current time and a headers time stamp
    TwistStampedArray::iterator it = std::lower_bound(
                    twists.begin(),
                    twists.end(),
                    time,
                    [](auto& twist, auto& time) {
                        return twist.header.stamp < time;
                    });
 
    // If the first time more than or equal is the first item in the array
    // then there is not a twist prior to the current time.
    // so a valid interpolated velocity cannot be calculated.
    if(it == twists.begin())
    {
        // ALl the times are greater than the current time
        ROS_ERROR("trimVelocityQueue there are no valid accelerations available");
        return false;
    }
    // There must be an item in the list prior to the iterator
    else
    {
        // Delete everything from the beginning up to (but not including) the node prior to the iterator
        (void)twists.erase(twists.begin(), std::prev(it, 1));
        return true;
    }
}

// Append velocity targets to the velocity queue. If the velocity targets are older than the newest velocity target queued
// the queued velocity targets are discarded
bool AccelerationPlanner::appendVelocityQueue(TwistStampedArray& current_twists, const TwistStampedArray& new_twists, const ros::Time& time)
{
    // Check for an empty array of new twists
    if(new_twists.empty())
    {
        ROS_ERROR("appendVelocityQueue handed empty twist array");
        return false;
    }

    // Find the first time from the new twists more than or equal to the current time, we won't keep anything prior
    TwistStampedArray::const_iterator first_valid_twist = std::lower_bound(new_twists.begin(), new_twists.end(), time,
                                                                           [](auto& twist, auto& time) { return twist.header.stamp < time; });

    // Check for a twist prior to the first twist that is more than or equal to the current time
    if(first_valid_twist != new_twists.begin())
    {
        // There is at least one element with a time less than current time
        // Make sure we get it so as to get a more accurate target interpolation value
        first_valid_twist = std::prev(first_valid_twist, 1);
    }
    
    // Search through to find where we should start inserting unless its empty
    if(!current_twists.empty())
    {
        // This is the time after which all velocities in the queue will be discarded
        ros::Time target_time = static_cast<ros::Time>(first_valid_twist->header.stamp);

        // Get an iterator to the first queue velocity with a timestamp more than or equal to the
        // first time we're appending from the new twists
        TwistStampedArray::iterator it = std::lower_bound(current_twists.begin(), current_twists.end(), target_time,
                                                          [](auto& twist, auto& time) { return twist.header.stamp < time; });

        // Check if its the last element before attempting to remove
        if(it != current_twists.end())
        {
            // Erase from the index to the end since the iterator must be somewhere in the the list
            (void)current_twists.erase(it, current_twists.end());
        }
    }

    // Copy all future targets into our buffer. All targets that were after these targets should be deleted now.
    std::for_each(first_valid_twist, new_twists.end(), [&](auto i){ current_twists.emplace_back(i); } );

    return true;
}

// Receive a new list of velocities commands and call appendVelocityQueue to insert them into the queue
// TODO: check that message is sorted properly, default to safe state if anythings wrong?
void AccelerationPlanner::processVelocityCommand(const iarc7_msgs::TwistStampedArrayStamped::ConstPtr& message)
{

    // Check for empty message
    if(message->data.empty())
    {
        ROS_WARN("processVelocityCommand passed an empty array of TwistStampedArray, not accepting");
        return;
    }

    // Cache time to make sure it stays the same for the proceeding function calls
    ros::Time current_time = ros::Time::now();

    appendVelocityQueue(velocity_targets_, message->data, current_time);
}

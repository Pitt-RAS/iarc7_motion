////////////////////////////////////////////////////////////////////////////
//
// Motion Point Interpolator
//
// Stores a queue of Motion Points commands from a topic. Interpolates
// between points when a Motion Point is requested.
//
////////////////////////////////////////////////////////////////////////////

// Associated header
#include "iarc7_motion/MotionPointInterpolator.hpp"

//System Headers
#include <algorithm>

using namespace Iarc7Motion;

// Construct the object need a node handle to register
// the subscriber for velocity targets
MotionPointInterpolator::MotionPointInterpolator(ros::NodeHandle& nh) :
motion_points_subscriber_(),
motion_point_targets_()
{
    motion_points_subscriber_ = nh.subscribe(
                    "motion_point_targets",
                    100,
                    &MotionPointInterpolator::processMotionPointArray,
                    this);

    // Put an initial time in the buffer so that buffer is never empty
    MotionPointStamped zero_state;
    zero_state.header.stamp = ros::Time::now();
    motion_point_targets_.emplace_back(zero_state);
}

// Called by a class user to get a target motion point
// Makes the call to trim the queue during the class
// Interpolates if there is more than one twist
void MotionPointInterpolator::getTargetMotionPoint(
                const ros::Time& current_time,
                MotionPointStamped& target_motion_point)
{

    // Trim velocity queue when done we will have one or two velocities available.
    trimMotionPointQueue(motion_point_targets_, current_time);

    if(motion_point_targets_.size() == 1)
    {
        target_motion_point = motion_point_targets_[0];
    }
    else
    {
        bool success = interpolateMotionPoints(
            motion_point_targets_[0],
            motion_point_targets_[1],
            target_motion_point,
            current_time);

        if(!success)
        {
            ROS_ERROR("Motion Point Interpolation failed, not filling out target motion point");
            return;
        }
    }
}

// Takes two twists and interpolates between their values
// using linear interpolation
bool MotionPointInterpolator::interpolateMotionPoints(
        MotionPointStamped& begin,
        MotionPointStamped& end,
        MotionPointStamped& interpolated,
        ros::Time time)
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
    double x = (time - begin_time).toSec() / (end_time - begin_time).toSec();

    // Interpolate linearly, do not interpolate orientation components as they are not supported
    interpolated.motion_point.pose.position.x = interpolate(x, begin.motion_point.pose.position.x, end.motion_point.pose.position.x);
    interpolated.motion_point.pose.position.y = interpolate(x, begin.motion_point.pose.position.y, end.motion_point.pose.position.y);
    interpolated.motion_point.pose.position.z = interpolate(x, begin.motion_point.pose.position.z, end.motion_point.pose.position.z);

    interpolated.motion_point.twist.linear.x = interpolate(x, begin.motion_point.twist.linear.x, end.motion_point.twist.linear.x);
    interpolated.motion_point.twist.linear.y = interpolate(x, begin.motion_point.twist.linear.y, end.motion_point.twist.linear.y);
    interpolated.motion_point.twist.linear.z = interpolate(x, begin.motion_point.twist.linear.z, end.motion_point.twist.linear.z);

    // z interpolation is left in for now since
    // yaw is not supported by any other way in the controller
    interpolated.motion_point.twist.angular.z = interpolate(x, begin.motion_point.twist.angular.z, end.motion_point.twist.angular.z);

    interpolated.motion_point.accel.linear.x = interpolate(x, begin.motion_point.accel.linear.x, end.motion_point.accel.linear.x);
    interpolated.motion_point.accel.linear.y = interpolate(x, begin.motion_point.accel.linear.y, end.motion_point.accel.linear.y);
    interpolated.motion_point.accel.linear.z = interpolate(x, begin.motion_point.accel.linear.z, end.motion_point.accel.linear.z);

    // Set the current header stamp
    interpolated.header.stamp = time;

    return true;
}

// Interpolate between arbitrary values and waypoints
// x is to be between 0-1 if the interpolation is to
// range from start to end.
double MotionPointInterpolator::interpolate(double x,
                                            double start,
                                            double end)
{
    return (end-start)*x +start;
}

// Trim the velocity queue so that there aren't old velocities in the queue
bool MotionPointInterpolator::trimMotionPointQueue(
    MotionPointStampedArray& motion_points,
    const ros::Time& time)
{
    // Check for empty array of twists
    if(motion_points.empty())
    {
        ROS_ERROR("trimMotionPointQueue detected no velocities available");
        return false;
    }

    // We want to get rid of all timestamps except for the one before
    // the current time and then also keep all going into the future.

    // Find the first time more than or equal
    // Uses a lambda (and an auto specifier) to compare
    //the current time and a header's time stamp
    MotionPointStampedArray::iterator it = std::lower_bound(
                    motion_points.begin(),
                    motion_points.end(),
                    time,
                    [](auto& motion_point, auto& time) {
                        return motion_point.header.stamp < time;
                    });
 
    // If the first time more than or equal is the first item in the array
    // then there is not a twist prior to the current time.
    // so a valid interpolated velocity cannot be calculated.
    if(it == motion_points.begin())
    {
        // ALl the times are greater than the current time
        ROS_ERROR("trimMotionPointQueue there are no valid motion points available");
        return false;
    }
    // There must be an item in the list prior to the iterator
    else
    {
        // Delete everything from the beginning up to (but not including) the node prior to the iterator
        (void)motion_points.erase(motion_points.begin(), std::prev(it, 1));
        return true;
    }
}

// Append velocity targets to the velocity queue. If the velocity targets are older than the newest velocity target queued
// the queued velocity targets are discarded
bool MotionPointInterpolator::appendMotionPointQueue(
    MotionPointStampedArray& current_motion_points,
    const MotionPointStampedArray& new_motion_points,
    const ros::Time& time)
{
    // Check for an empty array of new twists
    if(new_motion_points.empty())
    {
        ROS_ERROR("appendMotionPointQueue handed empty motion point array");
        return false;
    }

    // Find the first time from the new twists more than or equal to the current time
    // we won't keep anything prior
    MotionPointStampedArray::const_iterator first_valid_motion_point = std::lower_bound(
        new_motion_points.begin(),
        new_motion_points.end(),
        time,
        [](auto& motion_point, auto& time) { return motion_point.header.stamp < time; });

    // Check for a motion point prior to the first motion point
    // that is more than or equal to the current time
    if(first_valid_motion_point != new_motion_points.begin())
    {
        // There is at least one element with a time less than current time
        // Make sure we get it so as to get a more accurate target interpolation value
        first_valid_motion_point = std::prev(first_valid_motion_point, 1);
    }
    
    // Search through to find where we should start inserting unless its empty
    if(!current_motion_points.empty())
    {
        // This is the time after which all motion points in the queue will be discarded
        ros::Time target_time = static_cast<ros::Time>(first_valid_motion_point->header.stamp);

        // Get an iterator to the first queue velocity with a timestamp more than or equal to the
        // first time we're appending from the new twists
        MotionPointStampedArray::iterator it = std::lower_bound(
            current_motion_points.begin(),
            current_motion_points.end(),
            target_time,
            [](auto& motion_point, auto& time) { return motion_point.header.stamp < time; });

        // Check if its the last element before attempting to remove
        if(it != current_motion_points.end())
        {
            // Erase from the index to the end since the iterator must be somewhere in the the list
            (void)current_motion_points.erase(it, current_motion_points.end());
        }
    }

    // Copy all future targets into our buffer.
    // All targets that were after these targets should be deleted now.
    std::for_each(first_valid_motion_point,
                  new_motion_points.end(),
                  [&](auto i){ current_motion_points.emplace_back(i); } );

    return true;
}

// Receive a new list of velocities commands and call appendVelocityQueue to insert them into the queue
void MotionPointInterpolator::processMotionPointArray(
    const iarc7_msgs::MotionPointStampedArray::ConstPtr& message)
{

    for(MotionPointStampedArray::const_iterator checkMessageOrder = message->motion_points.begin();
        checkMessageOrder != message->motion_points.end() - 1; 
        checkMessageOrder++)
    {
        if(checkMessageOrder->header.stamp >= (checkMessageOrder+1)->header.stamp)
        {
            // messages[0] should have the earliest time
            ROS_ERROR("Messages out of order at time %lf and time %lf, rejecting entire message", 
            checkMessageOrder->header.stamp.toSec(), (checkMessageOrder+1)->header.stamp.toSec());
            
            return;
        }
    }

    // Check for empty message
    if(message->motion_points.empty())
    {
        ROS_WARN("processMotionPointArray passed an empty array" 
                  "of MotionPointStampedArray, not accepting");
        return;
    }

    // Cache time to make sure it stays the same for the proceeding function calls
    ros::Time current_time = ros::Time::now();

    appendMotionPointQueue(motion_point_targets_, message->motion_points, current_time);
}

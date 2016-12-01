////////////////////////////////////////////////////////////////////////////
//
// Acceleration Planner
//
// Implements receiving twist arrays and the acceleration ramps
//
////////////////////////////////////////////////////////////////////////////

#ifndef ACCLERATION_PLANNER_H
#define ACCLERATION_PLANNER_H

#include <ros/ros.h>
#include <algorithm>
#include <vector>
#include "iarc7_msgs/TwistStampedArrayStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "QuadVelocityController.hpp"

using geometry_msgs::TwistStamped;

namespace Iarc7Motion
{
    typedef std::vector<geometry_msgs::TwistStamped> TwistStampedArray;

    template<class T>
    class AccelerationPlanner
    {
    public:
        AccelerationPlanner() = delete;

        AccelerationPlanner(ros::NodeHandle& nh, T& velocity_controller);

        ~AccelerationPlanner() = default;

        // Don't allow the copy constructor or assignment.
        AccelerationPlanner(const AccelerationPlanner& rhs) = delete;
        AccelerationPlanner& operator=(const AccelerationPlanner& rhs) = delete;

        void update();

    private:

        void processVelocityCommand(const iarc7_msgs::TwistStampedArrayStamped::ConstPtr& message);

        void dispatchVelocity(const ros::TimerEvent&);

        static void trimVelocityQueue(TwistStampedArray& twists, const ros::Time& time);

        static TwistStamped interpolateTwists(TwistStamped& begin, TwistStamped& end, ros::Time time);

        ros::NodeHandle& nh_;

        ros::Timer dispatch_timer_;

        T& velocity_controller_;

        // Subscriber for uav_arm
        ros::Subscriber velocity_targets_subsriber_;

        TwistStampedArray velocity_targets_;
    };
}

using namespace Iarc7Motion;

template<class T>
AccelerationPlanner<T>::AccelerationPlanner(ros::NodeHandle& nh, T& velocity_controller) :
nh_(nh),
velocity_targets_subsriber_(),
velocity_targets_(),
velocity_controller_(velocity_controller)
{
    velocity_targets_subsriber_ = nh_.subscribe("movement_velocity_targets", 100, &AccelerationPlanner::processVelocityCommand, this);

    // Create the dispatch timer
    dispatch_timer_ = nh_.createTimer(ros::Duration(0.05), &AccelerationPlanner<T>::dispatchVelocity, this);

    // Put an initial time in the buffer
    TwistStamped zero_velocity;
    zero_velocity.header.stamp = ros::Time::now();
    velocity_targets_.emplace_back(zero_velocity);
}


template<class T>
void AccelerationPlanner<T>::dispatchVelocity(const ros::TimerEvent&)
{
    // Cache time to make sure it stays the same
    ros::Time current_time = ros::Time::now();

    // Trim velocity queue when done we will have one or two velocities available.
    trimVelocityQueue(velocity_targets_, current_time);

    TwistStamped target_twist;
    if(velocity_targets_.size() == 1U)
    {
        target_twist = velocity_targets_[0];
    }
    else
    {
        target_twist = interpolateTwists(velocity_targets_[0], velocity_targets_[1], current_time);
    }

    //velocity_controller_.setTargetVelocity(target_twist);
}

template<class T>
TwistStamped AccelerationPlanner<T>::interpolateTwists(TwistStamped& begin, TwistStamped& end, ros::Time time)
{
    TwistStamped interpolated;
    return interpolated;
}

template<class T>
void AccelerationPlanner<T>::trimVelocityQueue(TwistStampedArray& twists, const ros::Time& time)
{
    // We want to remove everything before the first time less than followed by a time more than or equal

    if(twists.empty())
    {
        ROS_ASSERT("trimVelocityQueue detected no velocities available");
    }

    // Find the first time more than or equal
    TwistStampedArray::iterator it = std::lower_bound(twists.begin(), twists.end(), time,
                                                      [](auto& twist, auto& time) { return twist.header.stamp < time; });
    

    if(it == twists.begin())
    {
        // ALl the times are greater than the current time
        ROS_ASSERT("trimVelocityQueue there are no valid acceleration profiles available");
    }
    else
    {
        // it is somwhere in the middle go delete stuff
        (void)twists.erase(twists.begin(), std::prev(it, 1));
    }
}

// Receive a new list of velocities commands and place them into our list properly
// Basic Operation:
// Assume sorted
// Search our velocities array until you find an item more than the earlies time in the message passed us
// Erase our velocities array from there on and add the velocities passed in
// TODO: check that message is sorted properly, default to safe state if anythings wrong?
template<class T>
void AccelerationPlanner<T>::processVelocityCommand(const iarc7_msgs::TwistStampedArrayStamped::ConstPtr& message)
{

    // Check for empty message
    if(message->data.empty())
    {
        ROS_WARN("processVelocityCommand passed an empty array of TwistStampedArray, not accepting");
        return;
    }

    // Cache time to make sure it stays the same
    ros::Time current_time = ros::Time::now();

    // Find the first time from the message more than or equal
    TwistStampedArray::const_iterator first_valid_twist = std::lower_bound(message->data.begin(), message->data.end(), current_time,
                                                                           [](auto& twist, auto& time) { return twist.header.stamp < time; });

    if(first_valid_twist == message->data.end())
    {
        ROS_ERROR("processVelocityCommand passed a TwistStampedArray with all old timestamps, rejecting");
        return;
    }
    else if(first_valid_twist != message->data.begin())
    {
        ROS_INFO("All velocities were flushed out");
        // There is at least one element with a time less than current time
        // All the stored velocities are invalid
        first_valid_twist = std::prev(first_valid_twist, 1);
        velocity_targets_.clear();
    }
    // Search through to find where we should start inserting unless its empty
    else if(!velocity_targets_.empty())
    {
        ros::Time target_time = static_cast<ros::Time>(first_valid_twist->header.stamp);

        // Get an iterator to the first time more than or equal to the first time we're appending from the message
        TwistStampedArray::iterator it = std::lower_bound(velocity_targets_.begin(), velocity_targets_.end(), target_time,
                                                          [](auto& twist, auto& time) { return twist.header.stamp < time; });

        // Check if its the last element before attempting to remove
        if(it != velocity_targets_.end())
        {
            // Erase from the index to the end since the iterator must be somewhere in the the list
            (void)velocity_targets_.erase(it, velocity_targets_.end());
        }
    }

    // Copy all future targets into our buffer. All targets that were after these targets should be deleted now.
    std::for_each(first_valid_twist, message->data.end(), [&](auto i){ velocity_targets_.emplace_back(i); } );
}

#endif // ACCLERATION_PLANNER_H

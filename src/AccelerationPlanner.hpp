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

        void trimVelocityQueue(ros::Time time);

        static TwistStamped interpolateTwists(TwistStamped& begin, TwistStamped& end, ros::Time time);

        ros::NodeHandle& nh_;

        ros::Timer dispatch_timer_;

        T& velocity_controller_;

        // Subscriber for uav_arm
        ros::Subscriber velocity_targets_subsriber_;

        typedef std::vector<geometry_msgs::TwistStamped> TwistArrayStamped;
        TwistArrayStamped velocity_targets_;
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

    trimVelocityQueue(current_time);

    TwistStamped target_twist;
    if(!velocity_targets_.empty())
    {
        if(velocity_targets_.size() == 1U)
        {
            target_twist = velocity_targets_[0];
        }
        else
        {
            target_twist = interpolateTwists(velocity_targets_[0], velocity_targets_[1], current_time);
        }
    }
    else
    {
        ROS_WARN("No velocity targets");
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
void AccelerationPlanner<T>::trimVelocityQueue(ros::Time time)
{
    // We want to remove everything before the first time less than followed by a time more than or equal

    if(velocity_targets_.empty())
    {
        // Nothing to do here
        return;
    }

    // Find the first time more than or equal
    TwistArrayStamped::iterator it = std::lower_bound(velocity_targets_.begin(), velocity_targets_.end(), time,
                                                      [](auto& twist, auto& time) { return twist.header.stamp < time; });
    

    if(it == velocity_targets_.begin())
    {
        // ALl the times are greater than the current time
        ROS_ASSERT("trimVelocityQueue there are no valid velocities available");
    }
    else
    {
        // it is somwhere in the middle go delete stuff
        (void)velocity_targets_.erase(velocity_targets_.begin(), std::prev(it, 1));
    }
}


// Receive a new list of velocities commands and place them into our list properly
// Basic Operation:
// Assume sorted
// Search our velocities array until you find an item more than the earlies time in the message passed us
// Erase our velocities array from there on and add the velocities passed in
// TODO: check that message is sorted properly, do not add if message is corrupt, default to safe state
template<class T>
void AccelerationPlanner<T>::processVelocityCommand(const iarc7_msgs::TwistStampedArrayStamped::ConstPtr& message)
{
    ros::Time last_message_time = velocity_targets_.back().header.stamp;

    // Check for empty message
    if(message->data.empty())
    {
        ROS_WARN("processVelocityCommand passed an empty array of TwistArrayStamped, not accepting");
        return;
    }

    // Don't search the list if the size is zero
    if(velocity_targets_.empty())
    {
        // Nothing to do in this case, just add the items and start the timer
    }
    // Do a simple check if the the velocity targets is one element long
    else if(velocity_targets_.size() == 1U && 
            velocity_targets_.front().header.stamp < static_cast<ros::Time>(message->data.front().header.stamp))
    {
        // Remove the first element since it's invalid
        velocity_targets_.erase(velocity_targets_.begin());
    }
    else
    {
        ros::Time target_time = static_cast<ros::Time>(message->data.front().header.stamp);

        // Get an iterator to the first time more than the first time from the message
        TwistArrayStamped::iterator it = std::lower_bound(velocity_targets_.begin(), velocity_targets_.end(), target_time,
                                                          [](auto& twist, auto& time) { return twist.header.stamp < time; });

        // Check if its the last element and decide whether to remove it
        if(it == velocity_targets_.end() && target_time <= it->header.stamp)
        {
            // Remove last element
            velocity_targets_.pop_back();
        }
        else
        {
            // Erase from the index to the end since the iterator must be somewhere in the the list
            (void)velocity_targets_.erase(it, velocity_targets_.end());
        }

    }

    // Copy all future targets into our buffer. All targets that were after these targets should be deleted now.
    std::for_each(message->data.begin(), message->data.end(), [&](auto i){ velocity_targets_.emplace_back(i); } );
}

#endif // ACCLERATION_PLANNER_H

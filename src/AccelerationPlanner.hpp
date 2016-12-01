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

    // Create the dispatch timer but do not call it
    dispatch_timer_ = nh_.createTimer(ros::Duration(0.0), &AccelerationPlanner<T>::dispatchVelocity, this, true);
    dispatch_timer_.stop();
}


template<class T>
void AccelerationPlanner<T>::dispatchVelocity(const ros::TimerEvent&)
{
    if(velocity_targets_.size() == 0)
    {
        ROS_WARN("Dispatch velocity called with no velocity targets");
    }
    else
    {
        //velocity_controller_.setTargetVelocity(velocity_targets.pop_front());
    }

    // Schedule the next velocity dispatch if it exists
    if(!velocity_targets_.empty())
    {
        dispatch_timer_.setPeriod(velocity_targets_.front().header.stamp - ros::Time::now());
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
        // Stop the timer since it's invalid now
        dispatch_timer_.stop();
        // Remove the first element since it's invalid
        velocity_targets_.erase(velocity_targets_.begin());
    }
    else
    {
        ros::Time target_time = static_cast<ros::Time>(message->data.front().header.stamp);

        // Get an iterator to the first time more than the first time from the message
        TwistArrayStamped::iterator it = std::upper_bound(velocity_targets_.begin(), velocity_targets_.end(),
                                                          target_time,
                                                          [](auto& a, auto& b) { return a < b.header.stamp; });

        // If its the first item stop the timer since we will need to reschedule it
        if(it == velocity_targets_.begin())
        {
            dispatch_timer_.stop();
        }

        // Check if its the last element and decide whether to remove it
        if(it == velocity_targets_.end() && target_time <= it->header.stamp)
        {
            // Remove last element
            (void)velocity_targets_.pop_back();
        }
        else
        {
            // Erase from the index to the end since the iterator must be somewhere in the the list
            (void)velocity_targets_.erase(it, velocity_targets_.end());
        }

    }

    // Copy all future targets into our buffer. All targets that were after these targets should be deleted now.
    std::for_each(message->data.begin(), message->data.end(), [&](auto i){ velocity_targets_.emplace_back(i); } );

    // If the dispatch timer wasn't already scheduled make sure to call it
    if(!dispatch_timer_.hasPending())
    {
        dispatch_timer_.setPeriod(velocity_targets_.front().header.stamp - ros::Time::now());
    }
}

#endif // ACCLERATION_PLANNER_H

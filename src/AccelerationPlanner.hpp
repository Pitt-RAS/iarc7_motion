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

        ros::NodeHandle& nh_;

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
}

// Receive a new list of velocities commands and place them into our list properly
// Basic Operation:
// Assume sorted
// Search our velocities array until you find an item more than the earlies time in the message passed us
// Erase our velocities array from there on and add the velocities passed in
template<class T>
void AccelerationPlanner<T>::processVelocityCommand(const iarc7_msgs::TwistStampedArrayStamped::ConstPtr& message)
{
    ros::Time last_message_time = velocity_targets_.back().header.stamp;

    // Check for empty message
    if(message->data.size() == 0U)
    {
        ROS_WARN("processVelocityCommand passed an empty array of TwistArrayStamped, not accepting");
    }

    // Don't search the list if the size is zero
    if(velocity_targets_.size() != 0U)
    {
        // Get the earliest time passed us
        ros::Time target_time = message->data.front().header.stamp;

        // Get an iterator to the first time more than ours
        TwistArrayStamped::iterator it = std::upper_bound(velocity_targets_.begin(), velocity_targets_.end(),
                                                          target_time, [](auto& a, auto& b) { return a < b.header.stamp; });

        // Check if its the last element because then we might still need to delete it if the target time is earlier
        if(it == velocity_targets_.end() && target_time <= it->header.stamp)
        {
            // Remove last element, start appending from the end
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
}

#endif // ACCLERATION_PLANNER_H

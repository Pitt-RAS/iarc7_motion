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
#include "gtest/gtest_prod.h"
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
        FRIEND_TEST(AccelerationPlannerTests, testTrimVelocityQueue);
        FRIEND_TEST(AccelerationPlannerTests, testAppendVelocityQueue);

        void processVelocityCommand(const iarc7_msgs::TwistStampedArrayStamped::ConstPtr& message);

        void dispatchVelocity(const ros::TimerEvent&);

        static bool trimVelocityQueue(TwistStampedArray& twists, const ros::Time& time);

        static bool appendVelocityQueue(TwistStampedArray& current_twists, const TwistStampedArray& new_twists, const ros::Time& time);

        static TwistStamped interpolateTwists(TwistStamped& begin, TwistStamped& end, ros::Time time);

        ros::NodeHandle& nh_;

        ros::Timer dispatch_timer_;

        T& velocity_controller_;

        // Subscriber for uav_arm
        ros::Subscriber velocity_targets_subsriber_;

        TwistStampedArray velocity_targets_;
    };

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
    bool AccelerationPlanner<T>::trimVelocityQueue(TwistStampedArray& twists, const ros::Time& time)
    {
        // We want to remove everything before the first time less than followed by a time more than or equal

        if(twists.empty())
        {
            ROS_ERROR("trimVelocityQueue detected no velocities available");
            return false;
        }

        // Find the first time more than or equal
        TwistStampedArray::iterator it = std::lower_bound(twists.begin(), twists.end(), time,
                                                          [](auto& twist, auto& time) { return twist.header.stamp < time; });
        

        if(it == twists.begin())
        {
            // ALl the times are greater than the current time
            ROS_ERROR("trimVelocityQueue there are no valid accelerations available");
            return false;
        }
        else
        {
            // it is somwhere in the middle go delete stuff
            (void)twists.erase(twists.begin(), std::prev(it, 1));
        }
    }

    template<class T>
    bool AccelerationPlanner<T>::appendVelocityQueue(TwistStampedArray& current_twists, const TwistStampedArray& new_twists, const ros::Time& time)
    {
        if(new_twists.empty())
        {
            ROS_ERROR("appendVelocityQueue handed empty twist array");
            return false;
        }

        // Find the first time from the new twists more than or equal
        TwistStampedArray::const_iterator first_valid_twist = std::lower_bound(new_twists.begin(), new_twists.end(), time,
                                                                               [](auto& twist, auto& time) { return twist.header.stamp < time; });

        if(first_valid_twist == new_twists.end())
        {
            ROS_ERROR("processVelocityCommand passed a TwistStampedArray with all old timestamps, rejecting");
            return false;
        }
        else if(first_valid_twist != new_twists.begin())
        {
            // There is at least one element with a time less than current time
            // Make sure we get it so as to get a more accurate target interpolation value
            first_valid_twist = std::prev(first_valid_twist, 1);
        }
        // Search through to find where we should start inserting unless its empty
        else if(!current_twists.empty())
        {
            ros::Time target_time = static_cast<ros::Time>(first_valid_twist->header.stamp);

            // Get an iterator to the first time more than or equal to the first time we're appending from the new twists
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

        appendVelocityQueue(velocity_targets_, message->data, current_time);
    }
} // End namespace Iarc7Motion

#endif // ACCLERATION_PLANNER_H

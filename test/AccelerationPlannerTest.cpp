// Bring in my package's API, which is what I'm testing
#include "iarc7_motion/AccelerationPlanner.hpp"
#include "iarc7_motion/QuadVelocityController.hpp"

// Bring in gtest
#include "gtest/gtest.h"


namespace Iarc7Motion
{
    TEST(AccelerationPlannerTests, testTrimVelocityQueue)
    {
        typedef Iarc7Motion::AccelerationPlanner<Iarc7Motion::QuadVelocityController> Planner;

        ros::Time::init();
        ros::Time current_time(ros::Time::now());

        Iarc7Motion::TwistStampedArray twists;
        // Create an array of 10 twists
        for(int32_t i = 0; i < 10; i++)
        {
            TwistStamped twist;
            twist.header.stamp = current_time + ros::Duration(i);
            twists.push_back(twist);
        }

        // Check general trimming
        // Check to make the sure the function doesn't incur an internal error
        EXPECT_TRUE(Planner::trimVelocityQueue(twists, current_time + ros::Duration(5.0)));
        // Check that the first time stamp before the barrier
        EXPECT_LT(twists[0].header.stamp, current_time + ros::Duration(5.0));
        // Check that the next time stamp is more than or equal to the barrier
        EXPECT_GE(twists[1].header.stamp, current_time + ros::Duration(5.0));

        // Make sure it errors when the divider time is less than the first time in twists
        EXPECT_FALSE(Planner::trimVelocityQueue(twists, current_time));

        // Make sure it does errors when the divider has the same time as the first element
        EXPECT_FALSE(Planner::trimVelocityQueue(twists, twists[0].header.stamp));

        // Make sure it returns the last item if the divider time is more than the last item in twists
        ros::Time last_time(twists.back().header.stamp);
        EXPECT_TRUE(Planner::trimVelocityQueue(twists, current_time + ros::Duration(20.0)));
        EXPECT_EQ(twists.size(), 1);
        EXPECT_EQ(twists[0].header.stamp, last_time);

        // Make sure it errors when handed an empty twist
        twists.empty();
        ASSERT_FALSE(Planner::trimVelocityQueue(twists, current_time));
    }

    TEST(AccelerationPlannerTests, testAppendVelocityQueue)
    {
        typedef Iarc7Motion::AccelerationPlanner<Iarc7Motion::QuadVelocityController> Planner;

        ros::Time::init();
        ros::Time current_time(ros::Time::now());

        Iarc7Motion::TwistStampedArray twists;
        Iarc7Motion::TwistStampedArray twists_append;
        // Create an array of 10 twists and ten twists to append
        for(int32_t i = 0; i < 10; i++)
        {
            TwistStamped twist;
            twist.header.stamp = current_time + ros::Duration(i);
            twists.push_back(twist);
            twist.header.stamp = current_time + ros::Duration(i+11.0);
            twists_append.push_back(twist);
        }

        // Make sure a list of old timestamps is rejected
        EXPECT_FALSE(Planner::appendVelocityQueue(twists_append, twists, current_time + ros::Duration(20.0)));

        // See if it will append all at the end as expected
        size_t expected_size = twists.size() + twists_append.size();
        EXPECT_TRUE(Planner::appendVelocityQueue(twists, twists_append, current_time));
        EXPECT_EQ(twists.size(), expected_size);

        for(int32_t i = 0; i < 10; i++)
        {
            // Test bottom half which should all be from twists
            EXPECT_EQ(twists[i].header.stamp, current_time + ros::Duration(i));

            // Test top half which should be all from tests_append
            EXPECT_EQ(twists[i + 10].header.stamp, current_time + ros::Duration(i + 11.0));
        }

        // See if it will overwrite in the middle as expected
        twists_append.pop_back();
        --expected_size;
        EXPECT_TRUE(Planner::appendVelocityQueue(twists, twists_append, current_time));
        EXPECT_EQ(twists.size(), expected_size);

        for(int32_t i = 0; i < 10; i++)
        {
            // Test bottom half which should all be from twists
            EXPECT_EQ(twists[i].header.stamp, current_time + ros::Duration(i));
        }

        for(int32_t i = 0; i < 9; i++)
        {
            // Test top half which should be all from tests_append
            EXPECT_EQ(twists[i + 10].header.stamp, current_time + ros::Duration(i + 11.0));
        }

        // See if it will not add the whole list if current time is more than the beginning
        // Regenerate twists and twists append
        twists.clear();
        twists_append.clear();
        // Create an array of 10 twists and ten twists to append
        for(int32_t i = 0; i < 10; i++)
        {
            TwistStamped twist;
            twist.header.stamp = current_time + ros::Duration(i);
            twists.push_back(twist);
            twist.header.stamp = current_time + ros::Duration(i+11.0);
            twists_append.push_back(twist);
        }

        // Should cut one element off of twists_append since it will keep the first one before the target time if it exists
        expected_size = twists.size() + twists_append.size() - 1;
        EXPECT_TRUE(Planner::appendVelocityQueue(twists, twists_append, current_time+ros::Duration(13.0)));
        EXPECT_EQ(twists.size(), expected_size);

        for(int32_t i = 0; i < 10; i++)
        {
            // Test bottom half which should all be from twists
            EXPECT_EQ(twists[i].header.stamp, current_time + ros::Duration(i));
        }

        for(int32_t i = 0; i < 8; i++)
        {
            // Test top half which should be all from tests_append 13.0 and up
            EXPECT_EQ(twists[i + 10].header.stamp, current_time + ros::Duration(i + 12.0));
        }

        // Make sure it errors when handed an empty twist
        twists_append.clear();
        ASSERT_FALSE(Planner::appendVelocityQueue(twists, twists_append, current_time));
    }
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

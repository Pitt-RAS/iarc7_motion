// Bring in my package's API, which is what I'm testing
#include "iarc7_motion/UAVMotionPointInterpolator.hpp"

// Bring in gtest
#include "gtest/gtest.h"


namespace Iarc7Motion
{
    TEST(UAVMotionPointInterpolatorTests, testTrimMotionPointQueue)
    {
        typedef Iarc7Motion::UAVMotionPointInterpolator Planner;

        ros::Time::init();
        ros::Time current_time(ros::Time::now());

        Iarc7Motion::UAVMotionPointStampedArray motion_points;
        // Create an array of 10 motion_points
        for(int32_t i = 0; i < 10; i++)
        {
            UAVMotionPointStamped motion_point;
            motion_point.header.stamp = current_time + ros::Duration(i);
            motion_points.push_back(motion_point);
        }

        // Check general trimming
        // Check to make the sure the function doesn't incur an internal error
        EXPECT_TRUE(Planner::trimMotionPointQueue(motion_points, current_time + ros::Duration(5.0)));
        // Check that the first time stamp before the barrier
        EXPECT_LT(motion_points[0].header.stamp, current_time + ros::Duration(5.0));
        // Check that the next time stamp is more than or equal to the barrier
        EXPECT_GE(motion_points[1].header.stamp, current_time + ros::Duration(5.0));

        // Make sure it errors when the divider time is less than the first time in motion points
        EXPECT_FALSE(Planner::trimMotionPointQueue(motion_points, current_time));

        // Make sure it does errors when the divider has the same time as the first element
        EXPECT_FALSE(Planner::trimMotionPointQueue(motion_points, motion_points[0].header.stamp));

        // Make sure it returns the last item if the divider time is more than the last item in motion points
        ros::Time last_time(motion_points.back().header.stamp);
        EXPECT_TRUE(Planner::trimMotionPointQueue(motion_points, current_time + ros::Duration(20.0)));
        EXPECT_EQ(motion_points.size(), 1);
        EXPECT_EQ(motion_points[0].header.stamp, last_time);

        // Make sure it errors when handed an empty motion point
        motion_points.empty();
        ASSERT_FALSE(Planner::trimMotionPointQueue(motion_points, current_time));
    }

    TEST(UAVMotionPointInterpolatorTests, testAppendMotionPointQueue)
    {
        typedef Iarc7Motion::UAVMotionPointInterpolator Planner;

        ros::Time::init();
        ros::Time current_time(ros::Time::now());

        Iarc7Motion::UAVMotionPointStampedArray motion_points;
        Iarc7Motion::UAVMotionPointStampedArray motion_points_append;
        // Create an array of 10 motion points and ten motion points to append
        for(int32_t i = 0; i < 10; i++)
        {
            UAVMotionPointStamped motion_point;
            motion_point.header.stamp = current_time + ros::Duration(i);
            motion_points.push_back(motion_point);
            motion_point.header.stamp = current_time + ros::Duration(i+11.0);
            motion_points_append.push_back(motion_point);
        }

        // Make sure a list of old timestamps is rejected except for the last
        size_t expected_size = motion_points.size() + 1;
        ros::Time expected_time = motion_points_append.back().header.stamp;
        EXPECT_TRUE(Planner::appendMotionPointQueue(motion_points, motion_points_append, current_time + ros::Duration(40.0)));
        EXPECT_EQ(motion_points.size(), expected_size);
        EXPECT_EQ(motion_points.back().header.stamp, expected_time);
        // Cleanup
        motion_points.pop_back();

        // See if it will append all at the end as expected
        expected_size = motion_points.size() + motion_points_append.size();
        EXPECT_TRUE(Planner::appendMotionPointQueue(motion_points, motion_points_append, current_time));
        EXPECT_EQ(motion_points.size(), expected_size);

        for(int32_t i = 0; i < 10; i++)
        {
            // Test bottom half which should all be from motion_points
            EXPECT_EQ(motion_points[i].header.stamp, current_time + ros::Duration(i));

            // Test top half which should be all from motion_points_append
            EXPECT_EQ(motion_points[i + 10].header.stamp, current_time + ros::Duration(i + 11.0));
        }

        // See if it will overwrite in the middle as expected
        motion_points_append.pop_back();
        --expected_size;
        EXPECT_TRUE(Planner::appendMotionPointQueue(motion_points, motion_points_append, current_time));
        EXPECT_EQ(motion_points.size(), expected_size);

        for(int32_t i = 0; i < 10; i++)
        {
            // Test bottom half which should all be from motion_points
            EXPECT_EQ(motion_points[i].header.stamp, current_time + ros::Duration(i));
        }

        for(int32_t i = 0; i < 9; i++)
        {
            // Test top half which should be all from motion_points_append
            EXPECT_EQ(motion_points[i + 10].header.stamp, current_time + ros::Duration(i + 11.0));
        }

        // See if it will not add the whole list if current time is more than the beginning
        // Regenerate motion_points and motion_points_append
        motion_points.clear();
        motion_points_append.clear();
        // Create an array of 10 motion points and ten motion points to append
        for(int32_t i = 0; i < 10; i++)
        {
            UAVMotionPointStamped motion_point;
            motion_point.header.stamp = current_time + ros::Duration(i);
            motion_points.push_back(motion_point);
            motion_point.header.stamp = current_time + ros::Duration(i+11.0);
            motion_points_append.push_back(motion_point);
        }

        // Should cut one element off of motion_points_append
        // since it will keep the first one before the target time if it exists
        expected_size = motion_points.size() + motion_points_append.size() - 1;
        EXPECT_TRUE(Planner::appendMotionPointQueue(motion_points, motion_points_append, current_time+ros::Duration(13.0)));
        EXPECT_EQ(motion_points.size(), expected_size);

        for(int32_t i = 0; i < 10; i++)
        {
            // Test bottom half which should all be from motion_points
            EXPECT_EQ(motion_points[i].header.stamp, current_time + ros::Duration(i));
        }

        for(int32_t i = 0; i < 8; i++)
        {
            // Test top half which should be all from motion_points_append 13.0 and up
            EXPECT_EQ(motion_points[i + 10].header.stamp, current_time + ros::Duration(i + 12.0));
        }

        // Make sure it errors when handed an empty motion_point
        motion_points_append.clear();
        ASSERT_FALSE(Planner::appendMotionPointQueue(motion_points, motion_points_append, current_time));
    }
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

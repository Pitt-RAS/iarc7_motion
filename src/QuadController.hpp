////////////////////////////////////////////////////////////////////////////
//
// Quad Controller
//
// Implements details involving the throttle controller
//
////////////////////////////////////////////////////////////////////////////

#include <ros/ros.h>
#include "FeedForwardPid.hpp"

namespace Iarc7Motion
{

    class QuadController
    {
    public:
        QuadController() = delete;

        QuadController(ros::NodeHandle& nh);

        ~QuadController() = default;

        // Don't allow the copy constructor or assignment.
        QuadController(const QuadController& rhs) = delete;
        QuadController& operator=(const QuadController& rhs) = delete;

        void init();

        void setCurrentHeight(const double height);
        void setCurrentPitchRollYaw(const double pitch, const double roll, const double yaw);

    private:
        void update(const ros::TimerEvent& time);

        void interpolatePositions(const double time_delta);

        ros::NodeHandle& nh_;

        ros::Publisher uav_control_;

        ros::Timer interpolation_timer_;

        FeedForwardPid throttle_pid_;
        FeedForwardPid pitch_pid_;
        FeedForwardPid roll_pid_;
        FeedForwardPid yaw_pid_;

        double interpolated_height_{0.0};
        double interpolated_pitch_{0.0};
        double interpolated_yaw_{0.0};
        double interpolated_roll_{0.0};
    };

}

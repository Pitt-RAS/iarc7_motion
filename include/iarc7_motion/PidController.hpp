#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

////////////////////////////////////////////////////////////////////////////
//
// PidController
//
// Class implement a PID loop
//
////////////////////////////////////////////////////////////////////////////

namespace Iarc7Motion
{

class PidController
{
public:

    PidController(double settings[6],
                  std::string debug_pid_name,
                  ros::NodeHandle& nh);

    PidController() = delete;
    ~PidController() = default;

    // Don't allow the copy constructor or assignment.
    PidController(const PidController& rhs) = delete;
    PidController& operator=(const PidController& rhs) = delete;

    void setSetpoint(double setpoint);

    // returns true on success
    bool __attribute__((warn_unused_result)) update(
            double current_value,
            const ros::Time& time,
            double& result,
            double derivative = std::numeric_limits<double>::quiet_NaN(),
            bool log_debug=true);

    void resetAccumulator();

    void reset();

private:
    double& p_gain_;
    double& i_gain_;
    double& d_gain_;

    double initialized_;
    double i_accumulator_;
    double last_current_value_;
    ros::Time last_time_;
    double setpoint_;

    double& i_accumulator_max_;
    double& i_accumulator_min_;
    double& i_accumulator_enable_threshold_;

    //Establishing the publisher for debuggin PID values
    ros::Publisher pid_value_publisher_;
};

}

#endif

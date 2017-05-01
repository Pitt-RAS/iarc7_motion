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

        PidController(double p_gain, double i_gain, double d_gain,
                      double i_accumulator_max, double i_accumulator_min,
                      double i_accumulator_enable_threshold);

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
                double derivative = std::numeric_limits<double>::quiet_NaN());

        void resetAccumulator();

    private:
        const double p_gain_;
        const double i_gain_;
        const double d_gain_;

        double initialized_;
        double i_accumulator_;
        double last_current_value_;
        ros::Time last_time_;
        double setpoint_;

        const double i_accumulator_max_;
        const double i_accumulator_min_;
        const double i_accumulator_enable_threshold_;
    };

}

#endif

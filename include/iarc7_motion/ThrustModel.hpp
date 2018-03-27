#ifndef IARC7_MOTION_THRUST_MODEL_HPP_
#define IARC7_MOTION_THRUST_MODEL_HPP_

#include <cmath>

#include <ros/ros.h>

namespace Iarc7Motion {

struct ThrustModel
{
    double small_thrust_epsilon;
    double model_mass;

    double response_lag;
    double thrust_min;
    double thrust_max;
    double voltage_min;
    double voltage_max;
    int num_thrust_points;
    int num_voltage_points;

    std::vector<double> thrust_to_voltage;

    double start_thrust = 0.0f;

    struct VoltageThrust {
        double voltage;
        double thrust;

        VoltageThrust(double voltage, double thrust)
            : voltage(voltage),
              thrust(thrust)
        {
        }

        friend std::ostream &operator<<(std::ostream &os, VoltageThrust& v) {
            return os << "[Voltage: " << v.voltage
                      << " Thrust: " << v.thrust
                      << "]";
        }
    };
    using VoltageThrustList = std::vector<VoltageThrust>;

    struct PossibleThrustFromThrust {
        double start_thrust;
        VoltageThrustList possible_thrusts;

        friend std::ostream &operator<<(std::ostream &os, PossibleThrustFromThrust& p) {
            os << "Start thrust: " << p.start_thrust
               << "\nPossible Thrusts: ";
            for(auto possible_thrust : p.possible_thrusts) {
                os << " " << possible_thrust;
            }
            return os;
        }
    };

    using PossibleThrustList = std::vector<PossibleThrustFromThrust>;
 
    PossibleThrustList voltage_to_jerk_mapping;

    double start_thrust_increment;

    ThrustModel(ros::NodeHandle& nh) {
        // Retrieve thrust model parameters
        ROS_ASSERT(nh.getParam("model_mass", model_mass));

        ROS_ASSERT(nh.getParam("thrust_model/response_lag",
                                       response_lag));
        ROS_ASSERT(nh.getParam("thrust_model/small_thrust_epsilon",
                                       small_thrust_epsilon));
        ROS_ASSERT(nh.getParam("thrust_model/thrust_to_voltage",
                                       thrust_to_voltage));

        ROS_ASSERT(nh.getParam("thrust_model/voltage_to_jerk/thrust_min",
                                       thrust_min));
        ROS_ASSERT(nh.getParam("thrust_model/voltage_to_jerk/thrust_max",
                                       thrust_max));
        ROS_ASSERT(nh.getParam("thrust_model/voltage_to_jerk/voltage_min",
                                       voltage_min));
        ROS_ASSERT(nh.getParam("thrust_model/voltage_to_jerk/voltage_max",
                                       voltage_max));

        XmlRpc::XmlRpcValue param_voltage_to_jerk_mapping;
        ROS_ASSERT(nh.getParam("thrust_model/voltage_to_jerk/mapping",
                               param_voltage_to_jerk_mapping));

        for(int i = 0; i < param_voltage_to_jerk_mapping.size(); i++)
        {
            PossibleThrustFromThrust possible_thrust_mapping;
            possible_thrust_mapping.start_thrust = static_cast<double>(param_voltage_to_jerk_mapping[i][0]);
            for(int j = 0; j < param_voltage_to_jerk_mapping[i][1].size(); j++){
                possible_thrust_mapping.possible_thrusts.emplace_back(
                    static_cast<double>(param_voltage_to_jerk_mapping[i][1][j][0]),
                    static_cast<double>(param_voltage_to_jerk_mapping[i][1][j][1]));
            }
            voltage_to_jerk_mapping.push_back(possible_thrust_mapping);
        }

        num_thrust_points = voltage_to_jerk_mapping.size();
        num_voltage_points = voltage_to_jerk_mapping[0].possible_thrusts.size();
        start_thrust_increment = (thrust_max - thrust_min) / (num_thrust_points-1);
    }

    double linearInterpolate(double x,
                             double x_i,
                             double x_f,
                             double y_i,
                             double y_f) {

        double a = ((y_f - y_i)/(x_f - x_i));
        double b = y_i - a*x_i;
        double result = a*x+b;

        return result;
    }

    double voltageFromThrust(double acceleration, int num_props, double /*height*/) {
        double desired_thrust =  model_mass * (acceleration / 9.81) / static_cast<double>(num_props);


        if(std::abs(desired_thrust - start_thrust) < small_thrust_epsilon){
            start_thrust = desired_thrust;
            return get_voltage_for_thrust(desired_thrust);
        }

        double start_thrust_index = start_thrust / start_thrust_increment;

        int bottom_thrust_index = std::min(std::max(static_cast<int>(std::floor(start_thrust_index)), 0), num_thrust_points-1);
        int top_thrust_index = std::min(std::max(bottom_thrust_index + 1, 0), num_thrust_points-1);

        PossibleThrustFromThrust bottom_thrusts = voltage_to_jerk_mapping[bottom_thrust_index];
        PossibleThrustFromThrust top_thrusts = voltage_to_jerk_mapping[top_thrust_index];

        double zero_voltage_thrust = linearInterpolate(
                                        start_thrust,
                                        bottom_thrusts.start_thrust,
                                        top_thrusts.start_thrust,
                                        bottom_thrusts.possible_thrusts[0].thrust,
                                        top_thrusts.possible_thrusts[0].thrust);

        if(zero_voltage_thrust >= desired_thrust) {
            start_thrust = desired_thrust;
            return 0.0f;
        }

        double last_final_thrust = zero_voltage_thrust;
        double voltage = voltage_max;

        for(int i = 1; i < num_voltage_points; i++) {

            // Interpolate between starting thrust rows while incrementing
            // by each array element, to find the first resulting thrust
            // greater than desired thrust.
            // Implicitly the thrust in the element before is the last thrust
            // that is less than the desired thrust.
            double current_final_thrust = linearInterpolate(
                start_thrust,
                bottom_thrusts.start_thrust,
                top_thrusts.start_thrust,
                bottom_thrusts.possible_thrusts[i].thrust, // End thrust bottom
                top_thrusts.possible_thrusts[i].thrust); // End thrust top

            if(current_final_thrust >= desired_thrust) {
                // Interpolate beteen voltages
                // because the voltages for each index are the same
                // in the top and bottom pair the bottom pair array is used
                // to get the voltage
                voltage = linearInterpolate(desired_thrust,
                                             last_final_thrust,
                                             current_final_thrust,
                                             bottom_thrusts.possible_thrusts[i].voltage,
                                             bottom_thrusts.possible_thrusts[i+1].voltage);
                break;
            }
            last_final_thrust = current_final_thrust;
        }

        start_thrust = desired_thrust;
        return std::min(std::max(voltage, voltage_min), voltage_max);
    }


    double get_voltage_for_thrust(double thrust) {
        double sum = 0;
        for(unsigned int i = 0; i < thrust_to_voltage.size(); i++) {
            double inc = thrust_to_voltage[i]*std::pow(thrust, thrust_to_voltage.size()-1-i);
            sum += inc;
        }
        return sum;
    }

};

}

#endif // include guard

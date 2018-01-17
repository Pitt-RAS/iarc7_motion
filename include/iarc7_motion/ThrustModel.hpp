#ifndef IARC7_MOTION_THRUST_MODEL_HPP_
#define IARC7_MOTION_THRUST_MODEL_HPP_

#include <cmath>

namespace Iarc7Motion {

struct ThrustModel
{
    // Overall scale factor in weight of drone (kg) / gravity / # motors
    double thrust_scale_factor;
    // A multiplier on the exponential used to determine ground effect
    double A_ge;
    // The divider of the height on the exponential used for ground effect
    double d0;

    // A polynomial representing the thrust mutliplier fit to voltage
    std::vector<double> voltage_polynomial;

    // Constants for the quadratic equation used to model thrust percentage
    // to thrust in relative drone units
    // We don't need an 'a' constant because otherwise thrust scale factor
    // would be a redundant parameter
    double throttle_b;
    double throttle_c;

    // Expected hover throttle
    double expected_hover_throttle;

    double controlPoly(double throttle) {
        return throttle*throttle + throttle_b*throttle + throttle_c;
    }

    double groundEffect(double height) {
        return 1 + A_ge * std::exp(-height / d0);
    }

    double vPoly(double voltage) {
        double v_poly = 0;
        for (size_t i = 0; i < voltage_polynomial.size(); i++) {
            size_t pow = voltage_polynomial.size() - 1 - i;
            v_poly += voltage_polynomial[i] * std::pow(voltage, pow);
        }
        return v_poly;
    }

    double throttleFromAccel(double accel,
                             double voltage,
                             double height) {
        // Find the thrust relative to the drone's mass
        double thrust = accel * thrust_scale_factor;

        // Apply voltage and ground effect compensation
        double C0 = thrust / vPoly(voltage) / groundEffect(height);

        // Solve the polynomial fit to thrust in percentage to thrust
        // in relative drone units
        double discriminant = std::pow(throttle_b, 2) - 4*(throttle_c-C0);
        return 0.5 * (-throttle_b + std::sqrt(discriminant));
    }

    void calibrate(double hover_throttle, double voltage, double height) {
        ROS_INFO("Calibrating thrust model with throttle = (%f), "
                 "voltage = (%f), "
                 "height = (%f)",
                 hover_throttle,
                 voltage,
                 height);
        double thrust = vPoly(voltage)
                      * groundEffect(height)
                      * controlPoly(hover_throttle);
        thrust_scale_factor = thrust / 9.8;
    }
};

}

#endif // include guard

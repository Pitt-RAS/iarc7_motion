#ifndef IARC7_MOTION_THRUST_MODEL_HPP_
#define IARC7_MOTION_THRUST_MODEL_HPP_

#include <cmath>

namespace Iarc7Motion {

struct ThrustModel
{
    double quadcopter_mass;
    double thrust_scale_factor;
    double A_ge;
    double d0;
    static constexpr const size_t VOLTAGE_POLYNOMIAL_DEGREE = 3;
    double voltage_polynomial[VOLTAGE_POLYNOMIAL_DEGREE + 1];
    double throttle_b;
    double throttle_c;

    double throttleFromAccel(double accel,
                             double voltage,
                             double height) {
        double thrust = accel * quadcopter_mass * thrust_scale_factor;
        double v_poly = 0;
        for (size_t i = 0; i <= VOLTAGE_POLYNOMIAL_DEGREE; i++) {
            size_t pow = VOLTAGE_POLYNOMIAL_DEGREE - i;
            v_poly += voltage_polynomial[i] * std::pow(voltage, pow);
        }
        double C0 = thrust / v_poly / (1 + A_ge * std::exp(-height / d0));
        double discriminant = std::pow(throttle_b, 2) - 4*(throttle_c-C0);
        return 0.5 * (-throttle_b + std::sqrt(discriminant));
    }
};

}

#endif // include guard

#ifndef IARC7_MOTION_THRUST_MODEL_HPP_
#define IARC7_MOTION_THRUST_MODEL_HPP_

#include <cmath>

namespace Iarc7Motion {

struct ThrustModel
{
    double thrust_scale_factor;
    double A_ge;
    double d0;
    std::vector<double> voltage_polynomial;
    double throttle_b;
    double throttle_c;

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
        double thrust = accel * thrust_scale_factor;
        double C0 = thrust / vPoly(voltage) / groundEffect(height);
        double discriminant = std::pow(throttle_b, 2) - 4*(throttle_c-C0);
        return 0.5 * (-throttle_b + std::sqrt(discriminant));
    }

    void calibrate(double hover_throttle, double voltage, double height) {
        double thrust = vPoly(voltage)
                      * groundEffect(height)
                      * controlPoly(hover_throttle);
        thrust_scale_factor = thrust / 9.8;
    }
};

}

#endif // include guard

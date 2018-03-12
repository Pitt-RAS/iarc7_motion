////////////////////////////////////////////////////////////////////////////
//
// Thrust Model Interpreter
//
// Uses the current estimated thrust and a desired thrust to
// calculate the voltage necessary to achieve the change within
// a specified timeperiod. Bilinear interpolation is used to find voltages
// between datapoints.
//
// Can also produce voltages that produce desired thrusts in the steady state
//
////////////////////////////////////////////////////////////////////////////

#ifndef THRUST_MODEL_INTERPRETER_H
#define THRUST_MODEL_INTERPRETER_H

// Automatically generated thrust model data
// Change this to use a different thrust model
//#include "apc_12_6_dynamic.h"
#include "unknown_10_45_dynamic.h"
//#include "6x4.5x2_dynamic.h"
//#include "6x4.5x3_dynamic.h"

const float start_thrust_increment = (thrust_max - thrust_min) / num_thrust_points;

float linear_interpolate(float x, float x_i, float x_f, float y_i, float y_f) {
    float a = ((y_f - y_i)/(x_f - x_i));
    float b = y_i - a*x_i;
    return a*x+b;
}

float get_voltage_for_jerk(float start_thrust, float desired_thrust) {

    start_thrust = constrain(start_thrust, thrust_min, thrust_max);

    float start_thrust_index = start_thrust / start_thrust_increment;

    // Arduino workaround so that ceil and floor produce integers
    // https://github.com/arduino/Arduino/issues/6714
    uint8_t bottom_thrust_index = constrain((floor)(start_thrust_index), 0, num_thrust_points);
    uint8_t top_thrust_index = constrain((ceil)(start_thrust_index), 0, num_thrust_points);


    float zero_voltage_thrust = linear_interpolate(start_thrust,
                                             voltage_to_jerk_mapping[bottom_thrust_index][0][0], // Start thrust bottom
                                             voltage_to_jerk_mapping[top_thrust_index][0][0], // Start thrust top
                                             voltage_to_jerk_mapping[bottom_thrust_index][1][1], // End thrust bottom
                                             voltage_to_jerk_mapping[top_thrust_index][1][1]); // End thrust top
    
    float last_final_thrust = zero_voltage_thrust;

    // If a voltage cannot be found that exceeds the desired thrust then
    // the maximum voltage will be used.
    float voltage = voltage_max;

    for(uint8_t i = 1; i < num_voltage_points; i++) {
        float current_final_thrust = linear_interpolate(start_thrust,
                                             voltage_to_jerk_mapping[bottom_thrust_index][0][0], // Start thrust bottom
                                             voltage_to_jerk_mapping[top_thrust_index][0][0], // Start thrust top
                                             voltage_to_jerk_mapping[bottom_thrust_index][i+1][1], // End thrust bottom
                                             voltage_to_jerk_mapping[top_thrust_index][i+1][1]); // End thrust top
        if(current_final_thrust >= desired_thrust) {
            // Calculate the linear interpolation and exit
            voltage = linear_interpolate(desired_thrust,
                                         last_final_thrust,
                                         current_final_thrust,
                                         voltage_to_jerk_mapping[bottom_thrust_index][i+1][0],
                                         voltage_to_jerk_mapping[bottom_thrust_index][i+1][0]);
            // Finish searching as the minimum voltage required to achieve
            // the desired thrust has been found.
            break;
        }
        last_final_thrust = current_final_thrust;
    }

    return constrain(voltage, voltage_min, voltage_max);

}

float get_voltage_for_thrust(float thrust) {
    float sum = 0;
    for(uint8_t i = 0; i <= thrust_to_voltage_order; i++) {
        sum += thrust_to_voltage[i]*pow(thrust, i);
    }
    return sum;
}

#endif // THRUST_MODEL_INTERPRETER_H

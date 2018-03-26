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

//#define DEBUG_PRINT

// Automatically generated thrust model data
// Change this to use a different thrust model
//#include "apc_12_6_dynamic.h"
//#include "apc_12_6_dynamic_r2.h"
//#include "unknown_10_45_dynamic.h"
#include "unknown_10_45_dynamic_r2.h"
//#include "6x4.5x2_dynamic_r2.h"
//#include "6x4.5x3_dynamic.h"

float get_voltage_for_thrust(float thrust);

const float start_thrust_increment = (thrust_max - thrust_min) / (num_thrust_points-1);

float linear_interpolate(float x, float x_i, float x_f, float y_i, float y_f) {

    float a = ((y_f - y_i)/(x_f - x_i));
    float b = y_i - a*x_i;
    float result = a*x+b;

    #ifdef DEBUG_PRINT
        Serial.print("a: "); Serial.println(a);
        Serial.print("b: "); Serial.println(b);
        Serial.print("x: "); Serial.print(x*1000);
        Serial.print(" x_i: "); Serial.print(x_i*1000);
        Serial.print(" x_f: "); Serial.print(x_f*1000);
        Serial.print(" y_i: "); Serial.print(y_i*1000);
        Serial.print(" y_f: "); Serial.print(y_f*1000);
        Serial.print(" r: "); Serial.println(result);
    #endif

    return result;
}

float get_voltage_for_jerk(float start_thrust, float desired_thrust, float& predicted_thrust) {

    if(abs(desired_thrust - start_thrust) < 0.01){
        predicted_thrust = desired_thrust;
        return get_voltage_for_thrust(desired_thrust);
    }

    #ifdef DEBUG_PRINT
        Serial.print("START THRUST: "); Serial.print(start_thrust); Serial.print("END: "); Serial.println(desired_thrust);
    #endif

    start_thrust = constrain(start_thrust, thrust_min, thrust_max);

    float start_thrust_index = start_thrust / start_thrust_increment;

    // Arduino workaround so that floor produces integers
    // https://github.com/arduino/Arduino/issues/6714
    uint8_t bottom_thrust_index = constrain((floor)(start_thrust_index), 0, num_thrust_points);
    uint8_t top_thrust_index = bottom_thrust_index + 1;

    float zero_voltage_thrust = linear_interpolate(start_thrust,
                                             voltage_to_jerk_mapping[bottom_thrust_index][0][0], // Start thrust bottom
                                             voltage_to_jerk_mapping[top_thrust_index][0][0], // Start thrust top
                                             voltage_to_jerk_mapping[bottom_thrust_index][1][1], // End thrust bottom
                                             voltage_to_jerk_mapping[top_thrust_index][1][1]); // End thrust top
    
    if(zero_voltage_thrust >= desired_thrust) {
        predicted_thrust = zero_voltage_thrust;
        return 0.0;
    }

    float last_final_thrust = zero_voltage_thrust;

    // If a voltage cannot be found that exceeds the desired thrust then
    // the maximum voltage will be used.
    float voltage = voltage_max;

    bool voltage_found = false;
    for(uint8_t i = 1; i < num_voltage_points; i++) {

        // Interpolate between starting thrust rows while incrementing
        // by each array element, to find the first resulting thrust
        // greater than desired thrust.
        // Implicitly the thrust in the element before is the last thrust
        // that is less than the desired thrust.

        #ifdef DEBUG_PRINT
            Serial.print("Interpolating thrust: "); Serial.println(i+1);
        #endif

        float current_final_thrust = linear_interpolate(start_thrust,
                                             voltage_to_jerk_mapping[bottom_thrust_index][0][0], // Start thrust bottom
                                             voltage_to_jerk_mapping[top_thrust_index][0][0], // Start thrust top
                                             voltage_to_jerk_mapping[bottom_thrust_index][i+1][1], // End thrust bottom
                                             voltage_to_jerk_mapping[top_thrust_index][i+1][1]); // End thrust top

        #ifdef DEBUG_PRINT
            Serial.println(current_final_thrust);
        #endif

        if(current_final_thrust >= desired_thrust) {
            #ifdef DEBUG_PRINT
                Serial.println("interpolating voltage");
            #endif
            voltage = linear_interpolate(desired_thrust,
                                               last_final_thrust,
                                               current_final_thrust,
                                               voltage_to_jerk_mapping[0][i][0],
                                               voltage_to_jerk_mapping[0][i+1][0]);

            #ifdef DEBUG_PRINT
                Serial.print("CURRENT FINAL: "); Serial.print(current_final_thrust); Serial.print("  "); Serial.println(i+1);
            #endif

            // Return the voltage found
            predicted_thrust = desired_thrust;
            voltage_found = true;
            break;
        }
        last_final_thrust = current_final_thrust;
    }

    if(!voltage_found) {
        predicted_thrust = last_final_thrust;
    }

    return constrain(voltage, voltage_min, voltage_max);

}

float get_voltage_for_thrust(float thrust) {
    float sum = 0;
    for(uint8_t i = 0; i <= thrust_to_voltage_order; i++) {
        sum += thrust_to_voltage[i]*pow(thrust, thrust_to_voltage_order-i);
    }
    return sum;
}

#undef DEBUG_PRINT

#endif // THRUST_MODEL_INTERPRETER_H

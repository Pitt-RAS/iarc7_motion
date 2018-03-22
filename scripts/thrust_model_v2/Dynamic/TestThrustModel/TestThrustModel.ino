#include <HX711.h>
#include <Servo.h> 

#include "ThrustModelInterpreter.h"

HX711 scale;
Servo esc;

int escPin = 5;
int minPulse = 900;
int minThrottle = 1000;
int maxThrottle = 2000;
int esc_update_rate = 1000000 / 50;

int currentPin = A2;
int adcCountOffset = 0;
// 5 V per 10 bit ADC count divided by 15 mV per amp from sensor datasheet
float ampsPerADCCount = -(5.0f/1024.0f) / 0.015f;

int voltagePin = A6;
float voltsPerADCCount = (5.0f/1024.0f)*(1.0f/(1.0f/5.7f))*(12.25f/12.11f)*(12.58f/12.7f);

int safetySwitchPin = 3;

const int current_limit = 50;
const int current_limit_frame_period = 100;
int current_limit_frame_position = 0;
unsigned long current_limit_accumulator = 0;

// In microseconds per percent
const unsigned long ramp_speed = 1000000 / 5;
// In microseconds
const unsigned long ramp_pause_at_full_throttle = 1000000;
const unsigned long total_ramp_duration = 100 * ramp_speed + ramp_pause_at_full_throttle;
unsigned long ramp_start_time = 0;

const unsigned long throttle_change_delay = 1500000; //3000000; //1500000;
const int max_throttle = 100; //70;

void setup() {
  Serial.begin(230400);
  Serial.setTimeout(50);

  pinMode(safetySwitchPin, INPUT);

  // Attach the the servo to the correct pin and set the pulse range
  esc.attach(escPin, minPulse, maxThrottle); 
  // Write a minimum value (most ESCs require this correct startup)
  esc.writeMicroseconds(minPulse);

  // Initialize the scale
  scale.begin(A4, A5);
  // Constant derived from calibration, includes factor of the disadvantged lever
  scale.set_scale(310556.71);
  // Zero the scale
  scale.tare();

  int current_sensor_calibration_runs = 20;
  uint32_t adc_counts = 0;
  // Calibrate the current sensor
  for(int i = 0; i < current_sensor_calibration_runs; i++) {
    adc_counts +=  analogRead(currentPin);
    delay(100);
  }
  adcCountOffset = adc_counts / current_sensor_calibration_runs;
}

int throttle = 0;
uint32_t lastUpdate = 0;
bool testing_active = false;
unsigned long last_esc_update = micros();
unsigned long scale_measurement_stamp = micros();

// Set last voltage to a small number initially to avoid divide by zero
float last_voltage = 0.1;

char buffer[100];

int command = 0 ;

void loop() {

  ////////////////////////////////////////////////////////////////////////////
  // Perform safety checks and start/stop tests
  ////////////////////////////////////////////////////////////////////////////

  if(digitalRead(safetySwitchPin)) {
    esc.writeMicroseconds(minPulse);
    throttle = 0;
    testing_active = false;
    Serial.println("STOP SAFETY");
    if(Serial.available() > 0) {
      Serial.parseInt();
    }
  }
  else if (current_limit_frame_position == current_limit_frame_period
           && current_limit_accumulator / current_limit_frame_period > current_limit) {
    esc.writeMicroseconds(minPulse);
    throttle = 0;
    testing_active = false;
    Serial.println("STOP CURRENT LIMIT");
    Serial.print("Current limit exceeded limit: "); Serial.print(current_limit);
    Serial.print(" Average over frame: "); Serial.println(current_limit_accumulator / current_limit_frame_period);
    delay(2000);
  }
  else {
    // Get throttle from the serial port
    static bool full_test = false;

    if (Serial.available() > 0) {
      // Read the new throttle value
      int raw = Serial.parseInt();
      command = constrain(raw, 0, 100);

      if(command == 0)
      {
        esc.writeMicroseconds(minPulse);
        testing_active = false;
        Serial.println("STOP COMMANDED");
        throttle = 0;
      }
      else if(command > 0 && command < 7){
        testing_active = true;
        ramp_start_time = micros();
      }
      else if(command == 7){
        // Going to run 1,2,3 in that order
        full_test = true;
      }
      else
      {
        Serial.println("UNKOWN COMMAND");
      }
    }

    //Serial.print(command); Serial.print(" "); Serial.print(full_test); Serial.print(" "); Serial.println(testing_active);

    if(command == 7 && full_test){
      command = 1;
      testing_active = true;
      ramp_start_time = micros();
    }
    else if(command == 1 && full_test && !testing_active){
      command = 2;
      testing_active = true;
      ramp_start_time = micros();
    }
    else if(command == 2 && full_test && !testing_active){
      command = 3;
      testing_active = true;
      ramp_start_time = micros();
    }
    else if(command == 3 && full_test && !testing_active){
      command = 4;
      testing_active = true;
      ramp_start_time = micros();
    }
    else if(command == 4 && full_test && !testing_active){
      command = 5;
      testing_active = true;
      ramp_start_time = micros();
    }
    else if(command == 5 && full_test && !testing_active){
      full_test = false;
    }
  }

  ////////////////////////////////////////////////////////////////////////////
  // Set throttle
  ////////////////////////////////////////////////////////////////////////////

  static float current_thrust;
  static float predicted_thrust = 0;
  static unsigned long current_thrust_timestamp;

  // Take response lag and add 10ms because esc updates go out every 20ms
  // if continuous uniform distribution is assumed this means the average lag
  // is 10ms (it is not valid to assume continuous uniform but its good enough for now)
  // Use the lower line if testing the non dynamic model
  //const unsigned long expected_response_lag_micros = (response_lag * 1000000) + 10000;
  const unsigned long expected_response_lag_micros = 20000;
  // const unsigned long expected_response_lag_micros = 0;

  if(esc.lastWriteStamp() - last_esc_update > 0) {

    static float last_thrust = 0;
    static int state = -1;
    static uint8_t triangles_completed = 0;
    static int triangles_per_test = 0;
    static int pauses_per_pause = 0;
    static float thrust_increment_rate = 0;
    static bool first_entry = true;

    if(testing_active) {

      float next_voltage = 0.0;

      if(command == 1 || command == 2 || command == 3 || command == 4) {
        const float start_thrust_increment_rate = 0.025;
        const float thrust_increment_rate_increment = 0.05;
        const float max_thrust_increment_rate = 0.9;
        const int min_triangles_per_test = 4;
        const int max_pauses = 5;
        const int start_up_settle_counts = 50;

        float thrust_bounds_margin;
        if(command == 1 || command == 2){
          thrust_bounds_margin = (thrust_max-thrust_min) * 0.25;
        }
        else{
          thrust_bounds_margin = (thrust_max-thrust_min) * 0.05;
        }

        if(first_entry) {
          first_entry = false;

          last_thrust = 0;
          triangles_completed = 0;
          thrust_increment_rate = start_thrust_increment_rate;
          predicted_thrust = 0.0;
          current_thrust = 0.0;
          current_thrust_timestamp =  expected_response_lag_micros + esc.lastWriteStamp();
          triangles_per_test = min_triangles_per_test;
          pauses_per_pause = max_pauses;
          state = -1;
        }
        if (state == -1) {
          static int pause_counter = 0;
          pause_counter++;
          current_thrust = thrust_min + thrust_bounds_margin;
          if(pause_counter > start_up_settle_counts) {
            pause_counter = 0;
            state = 0;
            Serial.println("START");
            Serial.println(command);
            Serial.println(thrust_increment_rate, 4);
          }
        }
        else if(state == 0) {
          current_thrust = last_thrust + thrust_increment_rate;
          if(current_thrust >= thrust_max - thrust_bounds_margin) {
            current_thrust = thrust_max - thrust_bounds_margin;
            state = 1;
          }
        }
       else if(state == 1) {
         static int pause_counter = 0;
         pause_counter++;
         if(pause_counter > pauses_per_pause) {
           pause_counter = 0;
           state = 2;
         }
       }
       else if(state == 2) {
          current_thrust = last_thrust - thrust_increment_rate;
          if(current_thrust <= thrust_min + thrust_bounds_margin) {
            current_thrust = thrust_min + thrust_bounds_margin;
            state = 3;
          }
       }
       else {
         static int pause_counter = 0;
         pause_counter++;
         if(pause_counter > pauses_per_pause) {
           pause_counter = 0;
           triangles_completed++;
           state = 0;
         }
       }

        if (triangles_completed >= triangles_per_test) {
          triangles_completed = 0;
          thrust_increment_rate = thrust_increment_rate * 2;
          //thrust_increment_rate += thrust_increment_rate_increment;
          //triangles_per_test = (thrust_increment_rate / thrust_increment_rate_increment) * min_triangles_per_test;
          //pauses_per_pause = min_triangles_per_test * max_pauses / triangles_per_test;

          if (thrust_increment_rate > max_thrust_increment_rate) {
            predicted_thrust = 0;
            current_thrust = 0;
            Serial.println("STOP");
            first_entry = true;
            testing_active = false;
          }
          else {
            Serial.println("RESPONSE END\nRESPONSE START");
            Serial.println(thrust_increment_rate);
          }
        }

        // Generate ramps for specified thrusts
        // Use the bottom line if testing the static model.
        if(command == 1 || command == 3) {
          float next_predicted_thrust;
          next_voltage = get_voltage_for_jerk(last_thrust, current_thrust, next_predicted_thrust);
          predicted_thrust = next_predicted_thrust;
        }
        else if(command == 2 || command == 4) {
          next_voltage = get_voltage_for_thrust(current_thrust);
          predicted_thrust = current_thrust;
        }

        last_thrust = current_thrust;
      }

      else if(command == 5 || command == 6) {
        const float start_thrust_increment_rate = 0.05;
        const float thrust_increment_rate_increment = 0.05;
        const float max_thrust_increment_rate = 0.45;
        const float thrust_bounds_margin = 0.05;
        const int min_triangles_per_test = 4;
        const int max_pauses = 150;
        const int start_up_settle_counts = 50;

        if(first_entry) {
            first_entry = false;

            last_thrust = 0;
            triangles_completed = 0;
            thrust_increment_rate = thrust_increment_rate_increment;
            predicted_thrust = 0.0;
            current_thrust = 0.0;
            current_thrust_timestamp =  expected_response_lag_micros + esc.lastWriteStamp();
            triangles_per_test = min_triangles_per_test;
            pauses_per_pause = max_pauses;
            state = -1;

            Serial.println("START");
            Serial.println(command);
            Serial.println(thrust_increment_rate);
        }
        if (state == -1) {
          static int pause_counter = 0;
          pause_counter++;
          current_thrust = thrust_min + thrust_bounds_margin;
          if(pause_counter > start_up_settle_counts) {
            pause_counter = 0;
            state = 0;
          }
        }
        else if(state == 0) {
          current_thrust = last_thrust + thrust_increment_rate;
          state = 1;
          if(current_thrust >= thrust_max - thrust_bounds_margin) {
            current_thrust = thrust_max - thrust_bounds_margin;
            state = 2;
          }
        }
        else if(state == 1) {
          static int pause_counter = 0;
          pause_counter++;
          if(pause_counter > pauses_per_pause) {
            pause_counter = 0;
            state = 0;
          }
        }
        else if(state == 2) {
          current_thrust = 0;
          predicted_thrust = 0;
          Serial.println("STOP");
          first_entry = true;
          testing_active = false;
        }

        // Generate ramps for specified thrusts
        // Use the bottom line if testing the static model.
        if(command == 5) {
          float next_predicted_thrust;
          next_voltage = get_voltage_for_jerk(last_thrust, current_thrust, next_predicted_thrust);
          predicted_thrust = next_predicted_thrust;
        }
        else if(command == 6) {
          next_voltage = get_voltage_for_thrust(current_thrust);
          predicted_thrust = current_thrust;
        }

        last_thrust = current_thrust;
      }

      throttle = 100.0f * next_voltage / last_voltage;

      // Write the throttle to the esc
      throttle = constrain(throttle, 0, 100);
      int micro_seconds = (((float)throttle / 100.0f) * (maxThrottle - minThrottle)) + minThrottle;

      // one period for the time till the next value to be written
      // Do not include one more period for the time required to rise
      // If the thrust model is too fast the thrust might be achieved before the target and
      // that can confuse the autocorrelation processing
      current_thrust_timestamp = expected_response_lag_micros + esc.lastWriteStamp();
      last_esc_update = esc.lastWriteStamp();
      esc.writeMicroseconds(micro_seconds);
    }
    else{
      esc.writeMicroseconds(minPulse);
    }
  }

  ////////////////////////////////////////////////////////////////////////////
  // Perform measurements
  ////////////////////////////////////////////////////////////////////////////

  static float thrust = 0;
  if(scale.is_ready()) {
    thrust = scale.get_units(1);
    scale_measurement_stamp = micros();
  }

  // Measure the current and battery voltage
  int analog_sensor_average = 20;
  uint32_t ADCCountsCurrent = 0;
  for(int i = 0; i < analog_sensor_average; i++) {
    ADCCountsCurrent += analogRead(currentPin);
    delayMicroseconds(50);
  }
  unsigned long current_stamp = micros();

  int voltage_sensor_average = 20;
  uint32_t ADCCountsVolts = 0;
  for(int i = 0; i < voltage_sensor_average; i++) {
    ADCCountsVolts += analogRead(voltagePin);
    delayMicroseconds(50);
  }
  unsigned long voltage_stamp = micros();
  
  int averagedADCCurrent = (int)(ADCCountsCurrent / analog_sensor_average) - adcCountOffset;
  unsigned int averagedADCVolts = ADCCountsVolts / voltage_sensor_average;
  float current = (float)averagedADCCurrent * ampsPerADCCount;
  float voltage = (float)averagedADCVolts * voltsPerADCCount;
  last_voltage = voltage;

  // Increment the current limit accumulator
  current_limit_accumulator += abs(current);
  current_limit_frame_position++;
  if(current_limit_frame_position > current_limit_frame_period) {
    current_limit_frame_position = 0;
    current_limit_accumulator = 0;
  }

  // Find the precise time of throttle writing
  static uint32_t last_actual_esc_write_stamp = 0;
  static int last_actual_esc_throttle = 0;
  uint32_t actual_esc_write_stamp = esc.lastWriteStamp();
  if(last_actual_esc_write_stamp != actual_esc_write_stamp)
  {
     last_actual_esc_write_stamp = actual_esc_write_stamp;
     last_actual_esc_throttle = throttle;
  }

  ////////////////////////////////////////////////////////////////////////////
  // Print to console
  ////////////////////////////////////////////////////////////////////////////

  unsigned long deltaT = micros() - lastUpdate;
  float updateRateHz = 1000000.0f/(float)deltaT;
  lastUpdate = micros();

  char thrust_str[15];
  dtostrf(thrust, 3, 3, thrust_str);
  char current_str[15];
  dtostrf(current, 3, 2, current_str);
  char voltage_str[15];
  dtostrf(voltage, 3, 2, voltage_str);
  char wattage_str[15];
  dtostrf(voltage*current, 3, 2, wattage_str);
  char current_thrust_str[15];
  dtostrf(current_thrust, 3, 3, current_thrust_str);
  char predicted_thrust_str[15];
  dtostrf(predicted_thrust, 3, 3, predicted_thrust_str);

  snprintf(buffer, sizeof(buffer), "%s, %ld, %s, %ld, %s, %ld, %s, %d, %ld, %d, %s, %s, %ld", thrust_str, 
                                                           scale_measurement_stamp - ramp_start_time,
                                                           current_str,
                                                           current_stamp - ramp_start_time,
                                                           voltage_str,
                                                           voltage_stamp - ramp_start_time,
                                                           wattage_str,
                                                           last_actual_esc_throttle,
                                                           last_actual_esc_write_stamp - ramp_start_time,
                                                           (int)updateRateHz,
                                                           current_thrust_str,
                                                           predicted_thrust_str,
                                                           current_thrust_timestamp - ramp_start_time);
  Serial.println(buffer);

}

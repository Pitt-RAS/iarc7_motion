#include <HX711.h>
#include <Servo.h> 

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

const unsigned long throttle_change_delay = 2500000; //1500000; //3000000; //1500000;
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
bool ramp = false;
unsigned long last_esc_update = micros();
unsigned long last_esc_write = micros();
unsigned long scale_measurement_stamp = micros();
void loop() {

  if(digitalRead(safetySwitchPin)) {
    esc.writeMicroseconds(minPulse);
    throttle = 0;
    ramp = false;
    Serial.println("STOP SAFETY");
    if(Serial.available() > 0) {
      Serial.parseInt();
    }
  }
  else if (current_limit_frame_position == current_limit_frame_period
           && current_limit_accumulator / current_limit_frame_period > current_limit) {
    esc.writeMicroseconds(minPulse);
    throttle = 0;
    ramp = false;
    Serial.println("STOP CURRENT LIMIT");
    Serial.print("Current limit exceeded limit: "); Serial.print(current_limit);
    Serial.print(" Average over frame: "); Serial.println(current_limit_accumulator / current_limit_frame_period);
    delay(2000);
  }
  else {
    // Get throttle from the serial port
    if (Serial.available() > 0) {
      // Read the new throttle value
      int raw = Serial.parseInt();
      int command = constrain(raw, 0, 100);
  
      if(command == 0)
      {
        esc.writeMicroseconds(minPulse);
        ramp = false;
        Serial.println("STOP COMMANDED");
        throttle = 0;
      }
      else
      {
        Serial.println("START");
        ramp = true;
        ramp_start_time = micros();
      }
    }
  }

  if(micros() - last_esc_update > esc_update_rate) {
    last_esc_update = micros();

    static int start_throttle = 0;
    static int end_throttle = 0;
    static unsigned long last_throttle_change = 0;
    if(ramp) {
      if(last_throttle_change == 0) {
        last_throttle_change = ramp_start_time;
      }
      int last_throttle = throttle;

      if(micros() - last_throttle_change > throttle_change_delay) {
        last_throttle_change = micros();

        if(throttle == end_throttle) {
          end_throttle += 10;
          if(end_throttle > max_throttle) {
            start_throttle += 10;
            end_throttle = start_throttle + 10;
            // If run is over
            if(start_throttle >= max_throttle) {
              start_throttle = 0;
              end_throttle = 0;
              ramp = false;
              Serial.println("STOP");
            }
          }
          throttle = start_throttle;
        }
        else if(throttle == start_throttle) {
          throttle = end_throttle;
        }
      }

      if(ramp) {
        throttle = constrain(throttle, 0, 100);
        int micro_seconds = (((float)throttle / 100.0f) * (maxThrottle - minThrottle)) + minThrottle;
        esc.writeMicroseconds(micro_seconds);
        last_esc_write = micros();

        if (last_throttle != throttle) {
          Serial.println("RESPONSE END\nRESPONSE START");
          Serial.print(last_throttle); Serial.print(", "); Serial.println(throttle);
        }
      }
      else {
        esc.writeMicroseconds(minPulse);
        last_esc_write = micros();
      }
    }
    else{
      start_throttle = 0;
      end_throttle = 0;
      last_throttle_change = 0;
      esc.writeMicroseconds(minPulse);
      last_esc_write = micros();
    }
  }

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

  // Print out everything
  unsigned long deltaT = micros() - lastUpdate;
  float updateRateHz = 1000000.0f/(float)deltaT;
  lastUpdate = micros();

  char buffer[250];
  char thrust_str[15];
  dtostrf(thrust, 3, 2, thrust_str);
  char current_str[15];
  dtostrf(current, 3, 2, current_str);
  char voltage_str[15];
  dtostrf(voltage, 3, 2, voltage_str);
  char wattage_str[15];
  dtostrf(voltage*current, 3, 2, wattage_str);
  //Serial.print((ADCCountsCurrent / analog_sensor_average)); Serial.print("  "); Serial.println(adcCountOffset);
  sprintf(buffer, "%s, %ld, %s, %ld, %s, %ld, %s, %d, %ld, %d", thrust_str, 
                                                           scale_measurement_stamp - ramp_start_time,
                                                           current_str,
                                                           current_stamp - ramp_start_time,
                                                           voltage_str,
                                                           voltage_stamp - ramp_start_time,
                                                           wattage_str,
                                                           last_actual_esc_throttle,
                                                           last_actual_esc_write_stamp - ramp_start_time,
                                                           (int)updateRateHz);
  Serial.println(buffer);

}

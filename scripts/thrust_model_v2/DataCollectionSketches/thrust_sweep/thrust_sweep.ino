#include <HX711.h>
#include <Servo.h> 

HX711 scale;
Servo esc;

int escPin = 5;
int minPulse = 900;
int minThrottle = 1000;
int maxThrottle = 2000;

int currentPin = A2;
int adcCountOffset = 0;
// 5 V per 10 bit ADC count divided by 15 mV per amp from sensor datasheet
float ampsPerADCCount = -(5.0f/1024.0f) / 0.015f;

int voltagePin = A6;
float voltsPerADCCount = (5.0f/1024.0f)*(1.0f/(1.0f/5.7f))*(23.84f/24.60f);

int safetySwitchPin = 3;

const int current_limit = 50;
const int current_limit_frame_period = 100;
int current_limit_frame_position = 0;
unsigned long current_limit_accumulator = 0;

// In microseconds per percent
const unsigned long ramp_speed = 1000000 / 5;
// In microseconds
const unsigned long ramp_pause_at_full_throttle = 1000000;
unsigned long ramp_start_time = 0;

void setup() {
  Serial.begin(250000);
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

  int current_sensor_calibration_runs = 100;
  uint32_t adc_counts = 0;
  // Calibrate the current sensor
  for(int i = 0; i < current_sensor_calibration_runs; i++) {
    adc_counts +=  analogRead(currentPin);
    delay(10);
  }
  adcCountOffset = adc_counts / current_sensor_calibration_runs;
}

int throttle = 0;
uint32_t lastUpdate = 0;
bool ramp = false;
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

  if(ramp) {
    static long last_timestamp = 0;
    if (throttle < 100 && micros() - last_timestamp > ramp_speed) {
      throttle++;
      last_timestamp = micros();
    }

    if(throttle == 100 && micros() - last_timestamp > ramp_pause_at_full_throttle){
      throttle = 0;
      ramp = false;
      Serial.println("STOP");
      esc.writeMicroseconds(minPulse);
    }
    else
    {
        throttle = constrain(throttle, 0, 100);
        int micro_seconds = (((float)throttle / 100.0f) * (maxThrottle - minThrottle)) + minThrottle;
        esc.writeMicroseconds(micro_seconds);
    }
  }

  float thrust = scale.get_units(1);
  unsigned long scale_measurement_stamp = micros();

  int analog_sensor_average = 3;
  unsigned int ADCCountsCurrent = 0;
  unsigned int ADCCountsVolts = 0;
  for(int i = 0; i < analog_sensor_average; i++) {
    ADCCountsCurrent += analogRead(currentPin);
    ADCCountsVolts += analogRead(voltagePin);
    delayMicroseconds(100);
  }
  int averagedADCCurrent = (ADCCountsCurrent / analog_sensor_average) - adcCountOffset;
  unsigned int averagedADCVolts = ADCCountsVolts / analog_sensor_average;
  float current = (float)averagedADCCurrent * ampsPerADCCount;
  float voltage = (float)averagedADCVolts * voltsPerADCCount;

  current_limit_accumulator += abs(current);
  current_limit_frame_position++;
  if(current_limit_frame_position > current_limit_frame_period) {
    current_limit_frame_position = 0;
    current_limit_accumulator = 0;
  }

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
  //sprintf(buffer, "T: %s \t C: %s \t V: %s \t W: %s \t Thr: %d \t Hz: %d", thrust_str, current_str, voltage_str, wattage_str, throttle, (int)updateRateHz);
  sprintf(buffer, "%s, %s, %s, %s, %d, %d, %ld", thrust_str, current_str, voltage_str, wattage_str, throttle, (int)updateRateHz, scale_measurement_stamp - ramp_start_time);
  Serial.println(buffer);

}

#include <Servo.h> 

Servo esc;

int escPin = 5;
int minPulse = 900;
int minThrottle = 1000;
int maxThrottle = 2000;

void setup() {
  Serial.begin(250000);
  Serial.setTimeout(50);

  pinMode(13, OUTPUT);

  // Attach the the servo to the correct pin and set the pulse range
  esc.attach(escPin, minPulse, maxThrottle); 
  // Write a minimum value (most ESCs require this correct startup)
  esc.writeMicroseconds(maxThrottle);

  delay(10000);
  esc.writeMicroseconds(minThrottle);
  digitalWrite(13, HIGH);

}

void loop() {

}

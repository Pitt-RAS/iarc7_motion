#include <Servo.h> 

Servo esc;

int escPin = 5;
int stopPulseRate = 1500;
int minPulseRate = 900;
int maxPulseRate = 2000;
int throttleChangeDelay = 0;

void setup() {
  
  Serial.begin(9600);
  Serial.setTimeout(500);
  
  // Attach the the servo to the correct pin and set the pulse range
  esc.attach(escPin, minPulseRate, maxPulseRate); 
  // Write a minimum value (most ESCs require this correct startup)
  esc.write(0);
  
}

void loop() {

  // Wait for some input
  if (Serial.available() > 0) {
    
    // Read the new throttle value
    int throttle = normalizeThrottle( Serial.parseInt() );
    
    // Print it out
    Serial.print("Setting throttle to: ");
    Serial.println(throttle);
    
    // Change throttle to the new value
    changeThrottle(throttle);
    
  }

  float value = analogRead(A2);

  float voltage = ((value*5.0f/1024.0f)-2.49+2.437-0.21);
  float difference = voltage - 2.23f;
  float current = difference / 0.015f;

  Serial.print(voltage); Serial.print('\t');
  Serial.print(difference); Serial.print('\t');
  Serial.println(current);
  delay(100);

}

void changeThrottle(int throttle) {
  
  // Read the current throttle value
  //int currentThrottle = readThrottle();
  
  // Are we going up or down?
  //int step = 1;
  //if( throttle < currentThrottle )
  //  step = -1;
  
  // Slowly move to the new throttle value 
  /*while( currentThrottle != throttle ) {
    esc.write(currentThrottle + step);
    currentThrottle = readThrottle();
    delay(throttleChangeDelay);
  }*/
  esc.write(throttle);
  
}

int readThrottle() {
  int throttle = esc.read();
  
  Serial.print("Current throttle is: ");
  Serial.println(throttle);
  
  return throttle;
}

// Ensure the throttle value is between 1000 - 2000
int normalizeThrottle(int value) {
  if( value < 0 )
    return 0;
  if( value > 180 )
    return 180;
  return value;
}

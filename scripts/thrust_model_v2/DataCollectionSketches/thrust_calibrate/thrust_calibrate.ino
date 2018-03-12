#include <HX711.h>

HX711 scale;

float units;
void setup() {
  Serial.begin(9600);
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);

  scale.begin(A4, A5);

  // Follow taring instructions as found on the README
  // https://github.com/bogde/HX711

  scale.set_scale();
  scale.tare();

  // Place a known weight on the scale after the LED turns on
  // You have 10 seconds
  digitalWrite(13, HIGH);
  delay(10000);
  digitalWrite(13, LOW);
  delay(500);

  // Call get_units 10 as instructed
  units = scale.get_units(10);
  // Assumes a known weight was used
  // The units of the weight can be any desired units
  // since its a linear scaling factor
  // For instance if a 1kg weight were used and
  // the desired units were kg set the literal below to 1
  scale.set_scale(units/2.583);

  // Turn the LED back to indicate success
  digitalWrite(13, HIGH);
}

boolean p = false;
void loop() {

  // Average 5 measurements and print to the terminal so that accuracy can be verified
  float weight = scale.get_units(5);
  Serial.print("kg: "); Serial.println(weight);
  Serial.print("scale factor: "); Serial.println(scale.get_scale());
}

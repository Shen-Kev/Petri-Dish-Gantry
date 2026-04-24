//WORKS WITH ARDUINO NANO ESP32 AND DS3218 270 DEGREE SERVO

#include <Arduino.h>

#include <Servo.h>

Servo myservo;

void moveToAngle(int angle);

void setup() {
  Serial.begin(9600);
  // Standard attach is (pin, 544, 2400). 
  // We override it to (pin, 500, 2500) for the DS3218 270 degree.
  myservo.attach(D3, 500, 2500); 
}

void loop() {
  moveToAngle(0);
  delay(3000);
  
  moveToAngle(90);
  delay(3000);
  
  moveToAngle(180);
  delay(3000);
  
  moveToAngle(270);
  delay(3000);
}

// Custom function to handle the 270-degree math
void moveToAngle(int angle) {
  // Map the desired angle (0-270) to the pulse width (500-2500)
  int pulse = map(angle, 0, 270, 500, 2500);
  myservo.writeMicroseconds(pulse);
  
  Serial.print("Angle: ");
  Serial.print(angle);
  Serial.print(" | Pulse: ");
  Serial.println(pulse);
}
//WORKS WITH ARDUINO NANO ESP32 AND 28BYJ-48 STEPPER MOTOR

#include <Arduino.h>
//Includes the Arduino Stepper Library

#include <Stepper.h>

//define Input pins of the Motor
#define OUTPUT1   D7                // Connected to the Blue coloured wire
#define OUTPUT2   D6                // Connected to the Pink coloured wire
#define OUTPUT3   D5                // Connected to the Yellow coloured wire
#define OUTPUT4   D4                // Connected to the Orange coloured wire

// Define the number of steps per rotation
const int stepsPerRotation = 2048;  // 28BYJ-48 has 2048 steps per rotation in full step mode as given in data sheet

// Initialize the stepper motor with the sequence of control pins OUTPUT1, OUTPUT3, OUTPUT2, IN4
// OUTPUT1 and OUTPUT3 are connected to one coil and OUTPUT2 and OUTPUT4 are connected to one Coil
Stepper myStepper(stepsPerRotation, OUTPUT1, OUTPUT3, OUTPUT2, OUTPUT4);  
void setup() {
  // Set the speed of the motor in RPM (adjust as needed)
  myStepper.setSpeed(15);  // 15 RPM
}

void loop() {
  // Rotate in One Direction and complete one complete rotation i.e 2048 steps
  myStepper.step(stepsPerRotation);  
  delay(1000);  // Delay between rotations
  // For Rotation in opposite direction provide the variable to the parameter with negative Sign
  myStepper.step(-stepsPerRotation);  
  delay(1000);  // Delay between rotations
}
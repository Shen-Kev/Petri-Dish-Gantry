// =============================================================================
//  POLAR GANTRY — FIRMWARE (SIMPLE FIXED-DELAY)
//  Target: Arduino Nano ESP32
//
//  Architecture:
//    moveTo() interpolates a straight line in Cartesian space at PATH_RESOLUTION
//    intervals. At each waypoint it writes the servo, then steps the stepper at
//    a fixed STEP_DELAY_MS until the target extension is reached. No buffering,
//    no ramping, no lookahead — structurally identical to the working test loop.
// =============================================================================

#include <Arduino.h>
#include <ezButton.h>
#include <Servo.h>

// =============================================================================
// PIN DEFINITIONS
// =============================================================================
#define LIMIT_SWITCH_PIN  D12
#define SERVO_PIN         D3
#define STEPPER_PIN1      D7
#define STEPPER_PIN2      D6
#define STEPPER_PIN3      D5
#define STEPPER_PIN4      D4

// =============================================================================
// GANTRY CONFIGURATION
// =============================================================================
const float STEPS_PER_REV  = 2048.0f;
const float TEETH_COUNT    = 12.0f;
const float TOOTH_PITCH_MM = 3.016f;
const float STEPS_PER_MM   = STEPS_PER_REV / (TEETH_COUNT * TOOTH_PITCH_MM);
                                              // ≈ 56.59 steps/mm

const float Y_LIMIT_SWITCH_OFFSET = 48.0f;   // mm, pivot → end effector at limit switch
const float MIN_EXTENSION         = Y_LIMIT_SWITCH_OFFSET;
const float MAX_EXTENSION         = 200.0f;  // mm, maximum physical reach

//const float X_OFFSET       = 8.0f;           // mm, pivot → work origin
const float X_OFFSET = 10.0f;
const float Y_OFFSET       = 124.0f;         // mm, pivot → work origin

const int   SERVO_MIN_PULSE = 500;            // µs → SERVO_MIN_DEG
const int   SERVO_MAX_PULSE = 2500;           // µs → SERVO_MAX_DEG
const float SERVO_MIN_DEG   = 0.0f;
const float SERVO_MAX_DEG   = 270.0f;        // MS24 is a 270° servo

// Servo coordinate offset: trig returns 90° for "straight forward",
// but the servo's physical neutral is at 135° (1500µs = midpoint of 270°).
const float SERVO_ANGLE_OFFSET = 45.0f;      // degrees

// Mounting trim: compensates for arm not being perfectly centred on the servo horn.
// Increase to rotate arm clockwise, decrease for CCW. Start at 0 and tune by eye.
const float SERVO_MOUNT_TRIM   = 3.0f;       // degrees

const float PATH_RESOLUTION = 5.0f;          // mm per interpolation waypoint KEY IS TO MAKE THIS LARGE ENOUGH, when it was only 2mm it semed to skip some steps

// =============================================================================
// STEP TIMING
// This is the only speed control. Decrease to go faster, increase if stalling.
// The working test used delay(2) = 2ms. Start here and tune down carefully.
// =============================================================================
const int STEP_DELAY_MS = 2;                 // ms per stepper step. 2 seems to be the limit

// =============================================================================
// OBJECTS & STATE
// =============================================================================
ezButton limitSwitch(LIMIT_SWITCH_PIN);
Servo    myservo;

struct State {
  float x, y;         // Current Cartesian work position (mm)
  long  stepperSteps; // Current absolute stepper step count
} current;

// =============================================================================
// PROTOTYPES
// =============================================================================
void homeStepper();
void ejectArm();
void moveTo(float targetX, float targetY);
void stepMotor(int direction);
void cartesianToPolar(float x, float y, float &r, float &theta);
int  servoAngleToMicros(float angleDeg);
void printStatus();
void testPlus();
void testDiamond();
void handleSerialInput();

// =============================================================================
// SETUP
// =============================================================================
void setup() {
  Serial.begin(115200);

  limitSwitch.setDebounceTime(50);
  myservo.attach(SERVO_PIN, SERVO_MIN_PULSE, SERVO_MAX_PULSE);

  //center servo
  myservo.writeMicroseconds(servoAngleToMicros(90.0f + SERVO_ANGLE_OFFSET + SERVO_MOUNT_TRIM));

  pinMode(STEPPER_PIN1, OUTPUT);
  pinMode(STEPPER_PIN2, OUTPUT);
  pinMode(STEPPER_PIN3, OUTPUT);
  pinMode(STEPPER_PIN4, OUTPUT);

  Serial.println("--- GANTRY ONLINE ---");
  Serial.println("Ejecting arm...");
  ejectArm();
  homeStepper();

  current.x = -X_OFFSET;                   // = -10.0
  current.y = MIN_EXTENSION - Y_OFFSET;    // = 48 - 124 = -76.0
  Serial.println("Moving to (0,0)...");
  moveTo(0.0f, 0.0f);                      // now has a real distance → actually moves
  printStatus();
  delay(1000);
  Serial.println("Entering Loop...");

}

// =============================================================================
// LOOP
// =============================================================================
void loop() {

  //testDiamond(); 
  handleSerialInput();

}

// =============================================================================
// SERIAL COMMAND HANDLER
// Usage: type "x,y" in Serial Monitor and press Enter. e.g. "40,0" or "-20,35"
// =============================================================================

void handleSerialInput() {
  Serial.println(F("Awaiting command (x,y):"));

  while (!Serial.available()) { delay(10); }

  String input = Serial.readStringUntil('\n');
  input.trim();
  input.replace("\r", "");  // explicitly strip any carriage return

  // Echo back what was actually received so you can verify
  Serial.print(F("Received: ["));
  Serial.print(input);
  Serial.println(F("]"));

  int commaIndex = input.indexOf(',');
  if (commaIndex == -1) {
    Serial.println(F("ERROR: Bad format. Use: x,y  (e.g. 40,0 or -20,35)"));
    return;
  }

  float x = input.substring(0, commaIndex).toFloat();
  float y = input.substring(commaIndex + 1).toFloat();

  Serial.print(F("Moving to X="));
  Serial.print(x, 2);
  Serial.print(F(", Y="));
  Serial.println(y, 2);

  moveTo(x, y);
  printStatus();
}


void testDiamond() {
  moveTo(40, 0);
  printStatus();
  delay(1000); 
  moveTo(0, 40);
  printStatus();
  delay(1000);
  moveTo(-40, 0);
  printStatus();
  delay(1000);
  moveTo(0, -40);
  printStatus();
  delay(1000);
}

void testPlus() {
  moveTo(0, 0);
  printStatus();
  delay(1000);

  moveTo(-40, 0);
  printStatus();
  delay(1000);

  moveTo(0, 0);
  printStatus();
  delay(1000);

  moveTo(40, 0);
  printStatus();
  delay(1000);

  moveTo(0, 0);
  printStatus();
  delay(1000);

  moveTo(0, 40);
  printStatus();
  delay(1000);

  moveTo(0, 0);
  printStatus();
  delay(1000);

  moveTo(0, -40);
  printStatus();
  delay(1000);
}
// =============================================================================
// CARTESIAN → POLAR CONVERSION
// Applies the physical pivot offset before converting.
// =============================================================================
void cartesianToPolar(float x, float y, float &r, float &theta) {
  float relX = x + X_OFFSET;
  float relY = y + Y_OFFSET;
  r     = sqrt(relX * relX + relY * relY);
  theta = atan2(relY, relX) * 180.0f / PI;
}

// =============================================================================
// SERVO ANGLE → MICROSECONDS
// Floating-point conversion to avoid integer truncation across the 270° range.
// =============================================================================
int servoAngleToMicros(float angleDeg) {
  float clamped = constrain(angleDeg, SERVO_MIN_DEG, SERVO_MAX_DEG);
  return (int)(SERVO_MIN_PULSE
    + (clamped - SERVO_MIN_DEG) / (SERVO_MAX_DEG - SERVO_MIN_DEG)
    * (float)(SERVO_MAX_PULSE - SERVO_MIN_PULSE));
}

// =============================================================================
// MOVETO
//
// Walks the straight line from current position to (targetX, targetY) in
// PATH_RESOLUTION steps. At each waypoint:
//   1. Convert interpolated (x, y) → polar (r, θ)
//   2. Write servo to θ (with coordinate and mounting offsets applied)
//   3. Step stepper at fixed STEP_DELAY_MS until arm reaches target extension
//
// This is structurally identical to the working test loop — no arrays,
// no ramping, no lookahead.
// =============================================================================
void moveTo(float targetX, float targetY) {
  float dx       = targetX - current.x;
  float dy       = targetY - current.y;
  float distance = sqrt(dx * dx + dy * dy);
  if (distance < 0.001f) return;

  int numWaypoints = (int)ceil(distance / PATH_RESOLUTION);

  for (int i = 1; i <= numWaypoints; i++) {
    float t       = (float)i / (float)numWaypoints;
    float interpX = current.x + dx * t;
    float interpY = current.y + dy * t;

    // Convert to polar
    float r, theta;
    cartesianToPolar(interpX, interpY, r, theta);

    // Write servo
    float angle = theta + SERVO_ANGLE_OFFSET + SERVO_MOUNT_TRIM;
    myservo.writeMicroseconds(servoAngleToMicros(angle));

    // Step stepper to match required extension
    float extension  = constrain(r - MIN_EXTENSION, 0.0f, MAX_EXTENSION - MIN_EXTENSION);
    long  targetSteps = (long)(extension * STEPS_PER_MM);

    while (current.stepperSteps != targetSteps) {
      int direction = (current.stepperSteps < targetSteps) ? 1 : -1;
      stepMotor(direction);
      current.stepperSteps += direction;
      delay(STEP_DELAY_MS);
    }
  }

  current.x = targetX;
  current.y = targetY;
}

// =============================================================================
// STEP MOTOR — Full-step drive (2 coils active, ~40% more torque than wave)
// =============================================================================
void stepMotor(int direction) {
  static int stepIndex = 0;
  stepIndex += (direction > 0) ? 1 : -1;
  if (stepIndex > 3) stepIndex = 0;
  if (stepIndex < 0) stepIndex = 3;

  switch (stepIndex) {
    case 0: digitalWrite(STEPPER_PIN1,HIGH);digitalWrite(STEPPER_PIN2,HIGH);digitalWrite(STEPPER_PIN3,LOW); digitalWrite(STEPPER_PIN4,LOW); break;
    case 1: digitalWrite(STEPPER_PIN1,LOW); digitalWrite(STEPPER_PIN2,HIGH);digitalWrite(STEPPER_PIN3,HIGH);digitalWrite(STEPPER_PIN4,LOW); break;
    case 2: digitalWrite(STEPPER_PIN1,LOW); digitalWrite(STEPPER_PIN2,LOW); digitalWrite(STEPPER_PIN3,HIGH);digitalWrite(STEPPER_PIN4,HIGH);break;
    case 3: digitalWrite(STEPPER_PIN1,HIGH);digitalWrite(STEPPER_PIN2,LOW); digitalWrite(STEPPER_PIN3,LOW); digitalWrite(STEPPER_PIN4,HIGH);break;
  }
}

// =============================================================================
// HOME STEPPER — Retract until limit switch triggers
// =============================================================================
void homeStepper() {
  Serial.println("Homing...");
  while (true) {
    limitSwitch.loop();
    if (limitSwitch.isPressed()) break;
    stepMotor(-1);
    delay(2);
  }
  current.stepperSteps = 0;
  Serial.println("Homing Complete.");
}

// =============================================================================
// EJECT ARM — Extend until limit switch releases
// =============================================================================
void ejectArm() {
  Serial.println("Ejecting...");

  //no matter what state the limit switch is in, eject for 1/2 rotation first in case its stuck on the switch
  for (int i = 0; i < STEPS_PER_REV/2; i++) {
    stepMotor(1);
    delay(2);
  } 

  while (true) {
    limitSwitch.loop();
    if (limitSwitch.isPressed() || limitSwitch.getState() == LOW) break;
    stepMotor(1);
    delay(2);
  }
  limitSwitch.loop();
  delay(500);
  Serial.println("Ejection Complete.");
}

// =============================================================================
// PRINT STATUS
// =============================================================================
void printStatus() {
  float relX         = current.x + X_OFFSET;
  float relY         = current.y + Y_OFFSET;
  float currentR     = sqrt(relX * relX + relY * relY);
  float currentTheta = atan2(relY, relX) * 180.0f / PI;

  Serial.println(F("\n--- GANTRY TELEMETRY ---"));
  Serial.print(F("Work Position:    X = ")); Serial.print(current.x, 2);
  Serial.print(F(" mm, Y = "));              Serial.print(current.y, 2);
  Serial.println(F(" mm"));
  Serial.print(F("Stepper Arm:      ")); Serial.print(currentR, 2);
  Serial.print(F(" mm extension ("));   Serial.print(current.stepperSteps);
  Serial.println(F(" steps)"));
  Serial.print(F("Servo Angle:      ")); Serial.print(currentTheta, 2);
  Serial.println(F(" degrees"));
  Serial.println(F("------------------------\n"));
}

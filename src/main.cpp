#include <Arduino.h>
#include <ezButton.h>
#include <Servo.h>


// --- PIN DEFINITIONS ---
#define LIMIT_SWITCH_PIN  D12
#define SERVO_PIN         D3
#define STEPPER_PIN1      D7
#define STEPPER_PIN2      D6
#define STEPPER_PIN3      D5
#define STEPPER_PIN4      D4

// --- GANTRY CONFIGURATION ---
const float STEPS_PER_REV = 2048.0;
const float TEETH_COUNT = 12.0;
const float TOOTH_PITCH_MM = 3.016;
const float STEPS_PER_MM = STEPS_PER_REV / (TEETH_COUNT * TOOTH_PITCH_MM);

const float MIN_EXTENSION = 37.5;  // mm from pivot to tool when homed
const float MAX_EXTENSION = 200.0; // mm maximum physical reach

// Pivot Offset: Where the servo is relative to your (0,0) work point.
// If (0,0) is 100mm directly in front of the servo:
const float X_OFFSET = 0.0;        
const float Y_OFFSET = 100.0;      

// Servo Limits
const int SERVO_MIN_PULSE = 500;   // Pulse for 0 degrees
const int SERVO_MAX_PULSE = 2500;  // Pulse for 180 degrees
const float SERVO_MIN_DEG = 0.0;
const float SERVO_MAX_DEG = 180.0;

const float PATH_RESOLUTION = 0.5; // mm segments for straight lines
const int STEP_DELAY_MIN = 2000;    // Microseconds (lower = faster stepper)

// --- OBJECTS & STATE ---
ezButton limitSwitch(LIMIT_SWITCH_PIN);
Servo myservo;

struct State {
  float x, y;          
  long stepperSteps;   
} current;

// --- FUNCTION PROTOTYPES ---
void homeStepper();
void moveTo(float targetX, float targetY);
void updateActuators(float r, float theta);
void stepMotor(int direction);
void printStatus();

void setup() {
  Serial.begin(115200);
  
  limitSwitch.setDebounceTime(50);
  myservo.attach(SERVO_PIN, SERVO_MIN_PULSE, SERVO_MAX_PULSE); 

  pinMode(STEPPER_PIN1, OUTPUT);
  pinMode(STEPPER_PIN2, OUTPUT);
  pinMode(STEPPER_PIN3, OUTPUT);
  pinMode(STEPPER_PIN4, OUTPUT);

  Serial.println("--- GANTRY ONLINE ---");
  homeStepper();
  
  // Initialize software position at 0,0
  current.x = 0;
  current.y = 0;
  moveTo(0, 0); 
}

void loop() {
  // Example: Draw a horizontal line from X=-40 to X=40 at Y=0
  moveTo(-40, 0);
  printStatus();
  delay(1000);
  printStatus();

  moveTo(40, 0);
  printStatus();

  delay(1000);
  
  // Move back to center
  moveTo(0, 0);
  printStatus();

  delay(5000);
}

/**
 * Calculates a straight-line path and moves in small increments
 */
void moveTo(float targetX, float targetY) {
  float dx = targetX - current.x;
  float dy = targetY - current.y;
  float distance = sqrt(dx * dx + dy * dy);
  
  int steps = ceil(distance / PATH_RESOLUTION);
  if (steps == 0) return;

  for (int i = 1; i <= steps; i++) {
    float interpX = current.x + (dx * (float)i / steps);
    float interpY = current.y + (dy * (float)i / steps);
    
    // Calculate position relative to the servo pivot
    float relX = interpX + X_OFFSET;
    float relY = interpY + Y_OFFSET;
    
    // Polar Conversion
    float targetR = sqrt(relX * relX + relY * relY);
    // atan2 returns angle from positive X axis. 
    // relY is "forward", relX is "side-to-side"
    float targetTheta = atan2(relY, relX) * 180.0 / PI; 
    
    updateActuators(targetR, targetTheta);
  }
  
  current.x = targetX;
  current.y = targetY;
}

/**
 * Synchronizes the stepper and servo movement
 */
void updateActuators(float r, float theta) {
  // 1. Extension Logic
  float extension = constrain(r - MIN_EXTENSION, 0, MAX_EXTENSION - MIN_EXTENSION);
  long targetSteps = (long)(extension * STEPS_PER_MM);
  
  // 2. Servo Logic (Constrained to 0-180)
  float angle = constrain(theta, SERVO_MIN_DEG, SERVO_MAX_DEG);
  int targetMicros = map(angle, SERVO_MIN_DEG, SERVO_MAX_DEG, SERVO_MIN_PULSE, SERVO_MAX_PULSE);

  // 3. Simultaneous step execution
  while (current.stepperSteps != targetSteps) {
    if (current.stepperSteps < targetSteps) {
      stepMotor(1);
      current.stepperSteps++;
    } else {
      stepMotor(-1);
      current.stepperSteps--;
    }
    
    // Update servo pulse during stepper transit for fluid motion
    myservo.writeMicroseconds(targetMicros);
    delayMicroseconds(STEP_DELAY_MIN); 
  }
  
  // Final sync for the servo if stepper didn't move
  myservo.writeMicroseconds(targetMicros);
}

void stepMotor(int direction) {
  static int stepIndex = 0;
  stepIndex = (direction > 0) ? (stepIndex + 1) : (stepIndex - 1);
  if (stepIndex > 3) stepIndex = 0;
  if (stepIndex < 0) stepIndex = 3;

  switch (stepIndex) {
    case 0: digitalWrite(STEPPER_PIN1, HIGH); digitalWrite(STEPPER_PIN2, LOW);  digitalWrite(STEPPER_PIN3, LOW);  digitalWrite(STEPPER_PIN4, LOW);  break;
    case 1: digitalWrite(STEPPER_PIN1, LOW);  digitalWrite(STEPPER_PIN2, HIGH); digitalWrite(STEPPER_PIN3, LOW);  digitalWrite(STEPPER_PIN4, LOW);  break;
    case 2: digitalWrite(STEPPER_PIN1, LOW);  digitalWrite(STEPPER_PIN2, LOW);  digitalWrite(STEPPER_PIN3, HIGH); digitalWrite(STEPPER_PIN4, LOW);  break;
    case 3: digitalWrite(STEPPER_PIN1, LOW);  digitalWrite(STEPPER_PIN2, LOW);  digitalWrite(STEPPER_PIN3, LOW);  digitalWrite(STEPPER_PIN4, HIGH); break;
  }
}

void homeStepper() {
  Serial.println("Homing...");
  while (true) {
    limitSwitch.loop();
    if (limitSwitch.isPressed() || limitSwitch.getState() == LOW) break;
    stepMotor(-1);
    delay(2);
  }
  current.stepperSteps = 0;
  Serial.println("Homing Complete.");
}


/**
 * Outputs the current Cartesian position and the raw actuator states.
 * Call this after a moveTo() or inside the loop for real-time tracking.
 */
void printStatus() {
  // Calculate current physical metrics based on the stored state
  float relX = current.x + X_OFFSET;
  float relY = current.y + Y_OFFSET;
  
  // Calculate the current Polar values for reporting
  float currentR = sqrt(relX * relX + relY * relY);
  float currentTheta = atan2(relY, relX) * 180.0 / PI;

  Serial.println(F("\n--- GANTRY TELEMETRY ---"));
  
  // End-Effector Position (Work Space)
  Serial.print(F("Work Position:    X = ")); Serial.print(current.x, 2);
  Serial.print(F(" mm, Y = ")); Serial.print(current.y, 2);
  Serial.println(F(" mm"));

  // Stepper State (Extension)
  Serial.print(F("Stepper Arm:      ")); Serial.print(currentR, 2);
  Serial.print(F(" mm extension (")); Serial.print(current.stepperSteps);
  Serial.println(F(" steps)"));

  // Servo State (Angle)
  Serial.print(F("Servo Angle:      ")); Serial.print(currentTheta, 2);
  Serial.println(F(" degrees"));
  
  Serial.println(F("------------------------\n"));
}
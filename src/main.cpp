// =============================================================================
//  POLAR GANTRY — FIRMWARE WITH ADAPTIVE TRAPEZOIDAL ACCELERATION
//  Target: Arduino Nano ESP32
//
//  Architecture Summary:
//    moveTo()      — Path planner. Pre-computes waypoints, splits at stepper
//                    direction reversals, dispatches sub-moves.
//    runSubMove()  — Executes one continuous-direction segment with a full
//                    trapezoidal (or triangle) ramp spanning the entire move.
//    computeDelay()— Returns the per-step µs delay for the ramp profile.
//    stepMotor()   — Wave-drive sequencer, unchanged from original.
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

const float MIN_EXTENSION  = 48.0f;          // mm from pivot at homed position
const float MAX_EXTENSION  = 200.0f;         // mm maximum physical reach

const float X_OFFSET       = 0.0f;           // mm, pivot → work origin
const float Y_OFFSET       = 124.0f;         // mm, pivot → work origin
const int   SERVO_MIN_PULSE = 500;            // µs  → SERVO_MIN_DEG
const int   SERVO_MAX_PULSE = 2500;           // µs  → SERVO_MAX_DEG
const float SERVO_MIN_DEG   = 0.0f;
const float SERVO_MAX_DEG   = 270.0f;
const float SERVO_ANGLE_OFFSET = 45.0f; // trig "forward" = 90°, servo neutral = 135°

const float PATH_RESOLUTION = 0.5f;          // mm per interpolation segment

// =============================================================================
// SPEED / RAMPING CONFIGURATION
// All three values are the primary tuning knobs. Start conservative and
// increase STEP_DELAY_CRUISE incrementally once motion is confirmed stable.
//
//   STEP_DELAY_START  : delay at step 0 (slowest). Must be slow enough that
//                       the motor can move from a dead stop without stalling.
//   STEP_DELAY_CRUISE : delay at full speed (fastest, lowest µs value).
//                       Decrease to go faster, increase if stalling at speed.
//   ACCEL_STEPS       : number of steps over which to ramp START → CRUISE.
//                       More steps = gentler ramp = safer for heavy loads.
// =============================================================================
const int STEP_DELAY_START  = 8000;          // µs  (~125 steps/sec)
const int STEP_DELAY_CRUISE = 1500;          // µs  (~667 steps/sec)
const int ACCEL_STEPS       = 200;           // steps across the ramp

// =============================================================================
// WAYPOINT BUFFER
//
// One waypoint per PATH_RESOLUTION segment. Worst case:
//   MAX_EXTENSION / PATH_RESOLUTION = 200 / 0.5 = 400 waypoints.
// 512 provides headroom for diagonal moves. Stored as a static array
// (lives in .bss, not on the stack) to avoid heap fragmentation.
//
// Memory cost: 512 × (4 + 2) bytes = 3072 bytes — fine on Nano ESP32 (320 KB).
// =============================================================================
#define MAX_WAYPOINTS 512

struct Waypoint {
  long stepperTarget; // Absolute stepper step position for this waypoint
  int  servoMicros;   // Servo pulse width (µs) for this waypoint
};

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
void runSubMove(Waypoint* wps, int wpStart, int wpEnd,
                long subTotalSteps, int direction);
int  computeDelay(long stepIndex, long totalSteps);
void stepMotor(int direction);
void cartesianToPolar(float x, float y, float &r, float &theta);
int  servoAngleToMicros(float angleDeg);
void printStatus();

// =============================================================================
// SETUP
// =============================================================================
void setup() {
  Serial.begin(115200);

  limitSwitch.setDebounceTime(50);
  myservo.attach(SERVO_PIN, SERVO_MIN_PULSE, SERVO_MAX_PULSE);

  pinMode(STEPPER_PIN1, OUTPUT);
  pinMode(STEPPER_PIN2, OUTPUT);
  pinMode(STEPPER_PIN3, OUTPUT);
  pinMode(STEPPER_PIN4, OUTPUT);

  Serial.println("--- GANTRY ONLINE ---");
  ejectArm();
  homeStepper();

  current.x = 0.0f;
  current.y = 0.0f;
  moveTo(0.0f, 0.0f);
}

// =============================================================================
// LOOP
// =============================================================================
void loop() {
  moveTo(-40, 0);
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

  moveTo(0, -40);
  printStatus();
  delay(1000);

  moveTo(0, 0);
  printStatus();
  delay(1000);

  //blink the arduino nano's LED
  for (int i = 0; i < 5; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(200);
    digitalWrite(LED_BUILTIN, LOW);
    delay(200);
  }

}

// =============================================================================
// CARTESIAN → POLAR CONVERSION
//
// Applies the physical pivot offset so the work origin (0, 0) sits at the
// correct location relative to the servo pivot point.
// =============================================================================
void cartesianToPolar(float x, float y, float &r, float &theta) {
  float relX = x + X_OFFSET;
  float relY = y + Y_OFFSET;
  r     = sqrt(relX * relX + relY * relY);
  theta = atan2(relY, relX) * 180.0f / PI;
}

// =============================================================================
// SERVO ANGLE → MICROSECONDS
//
// Floating-point replacement for map() to avoid integer truncation error
// across the full 0–180° → 500–2500µs range.
// =============================================================================
int servoAngleToMicros(float angleDeg) {
  float clamped = constrain(angleDeg, SERVO_MIN_DEG, SERVO_MAX_DEG);
  return (int)(SERVO_MIN_PULSE
    + (clamped - SERVO_MIN_DEG) / (SERVO_MAX_DEG - SERVO_MIN_DEG)
    * (float)(SERVO_MAX_PULSE - SERVO_MIN_PULSE));
}

// =============================================================================
// ADAPTIVE TRAPEZOIDAL DELAY
//
// Returns the step delay (µs) for step [stepIndex] within a sub-move of
// [totalSteps] stepper steps total.
//
// Two profiles are chosen automatically:
//
//   TRIANGLE  (totalSteps ≤ 2 × ACCEL_STEPS)
//     The move is too short to reach cruise speed. The motor ramps up to a
//     peak speed proportional to move length, then ramps back down.
//     This prevents the motor from being commanded at cruise speed for a
//     move it physically cannot accelerate into.
//
//              Delay
//                ^
//   STEP_START  |\ /|
//               | X |
//   (peak)      |/ \|
//               +---+—> Steps
//
//   TRAPEZOID (totalSteps > 2 × ACCEL_STEPS)
//     Full accel → cruise → decel.
//
//              Delay
//                ^
//   STEP_START  |\         /|
//               | \_______/ |
//   CRUISE      |           |
//               +-----------+—> Steps
//               [A]  [C]  [D]
// =============================================================================
int computeDelay(long stepIndex, long totalSteps) {
  if (totalSteps <= 0) return STEP_DELAY_START;

  if (totalSteps <= 2L * ACCEL_STEPS) {

    // --- TRIANGLE PROFILE ---
    // Peak speed scales with how much of the full ramp we can fit.
    // A move half as long as the ramp gets half the speed gain.
    long  halfSteps = max(1L, totalSteps / 2);
    float peakFrac  = (float)halfSteps / (float)ACCEL_STEPS; // 0.0 – 1.0
    int   peakDelay = (int)((float)STEP_DELAY_START
                      - (float)(STEP_DELAY_START - STEP_DELAY_CRUISE) * peakFrac);
    float t;
    if (stepIndex <= halfSteps) {
      t = (float)stepIndex / (float)halfSteps;
    } else {
      t = (float)(totalSteps - stepIndex) / (float)(totalSteps - halfSteps);
    }
    return (int)((float)STEP_DELAY_START + (float)(peakDelay - STEP_DELAY_START) * t);

  } else {

    // --- TRAPEZOID PROFILE ---
    if (stepIndex < ACCEL_STEPS) {
      // Acceleration: START → CRUISE
      float t = (float)stepIndex / (float)ACCEL_STEPS;
      return (int)((float)STEP_DELAY_START
                   + (float)(STEP_DELAY_CRUISE - STEP_DELAY_START) * t);

    } else if (stepIndex >= totalSteps - ACCEL_STEPS) {
      // Deceleration: CRUISE → START
      // t = 1.0 at the first decel step (still at cruise), 0.0 at the last step (back to start).
      float t = (float)(totalSteps - 1 - stepIndex)
                / (float)max(1, ACCEL_STEPS - 1);
      return (int)((float)STEP_DELAY_START
                   + (float)(STEP_DELAY_CRUISE - STEP_DELAY_START) * t);

    } else {
      // Cruise
      return STEP_DELAY_CRUISE;
    }
  }
}

// =============================================================================
// RUN SUB-MOVE
//
// Executes waypoints [wpStart, wpEnd] inclusive, all in the same stepper
// direction, using a single continuous ramp across subTotalSteps steps.
//
// Key invariant: current.stepperSteps is always up to date, so
//   abs(wps[i].stepperTarget - current.stepperSteps)
// gives the exact number of steps to take to reach waypoint i from wherever
// the motor currently is.
//
// Servo updates: each waypoint is only PATH_RESOLUTION (0.5 mm) apart, so
// the angular change between consecutive waypoints is <0.3°. Writing the
// servo directly to each waypoint's target is indistinguishable from
// interpolation at this resolution.
// =============================================================================
void runSubMove(Waypoint* wps, int wpStart, int wpEnd,
                long subTotalSteps, int direction) {
  if (subTotalSteps == 0 || wpStart > wpEnd) {
    // No stepper motion — still sync servo to final waypoint angle
    if (wpStart <= wpEnd) myservo.writeMicroseconds(wps[wpEnd].servoMicros);
    return;
  }

  long globalStep = 0; // Counts every step taken in this entire sub-move

  for (int i = wpStart; i <= wpEnd; i++) {
    // Update servo before the stepper steps for this waypoint.
    // Because segments are 0.5 mm, the servo leads the stepper by at most
    // one tiny increment — this is imperceptible and keeps the path accurate.
    myservo.writeMicroseconds(wps[i].servoMicros);

    long segSteps = abs(wps[i].stepperTarget - current.stepperSteps);

    for (long s = 0; s < segSteps; s++) {
      int stepDelay = computeDelay(globalStep, subTotalSteps);
      stepMotor(direction);
      current.stepperSteps += direction;
      globalStep++;
      delayMicroseconds(stepDelay);
    }
  }
}

// =============================================================================
// MOVETO — Main Motion Planner
//
// Phase 1 — Pre-compute
//   Walks the Cartesian straight line in PATH_RESOLUTION steps and converts
//   each interpolated point to (stepperTarget, servoMicros) via polar math.
//
// Phase 2 — Sub-move dispatch
//   Scans the waypoint list for stepper direction reversals. Each continuous
//   same-direction run is dispatched to runSubMove() as a single ramp.
//   Because every sub-move starts and ends near STEP_DELAY_START, the motor
//   is always nearly stopped before a direction change — no extra logic needed.
// =============================================================================
void moveTo(float targetX, float targetY) {
  float dx       = targetX - current.x;
  float dy       = targetY - current.y;
  float distance = sqrt(dx * dx + dy * dy);
  if (distance < 0.001f) return; // Already there

  int numWaypoints = (int)ceil(distance / PATH_RESOLUTION);
  if (numWaypoints > MAX_WAYPOINTS) {
    Serial.print(F("WARNING: Move clamped from "));
    Serial.print(numWaypoints);
    Serial.print(F(" to "));
    Serial.print(MAX_WAYPOINTS);
    Serial.println(F(" waypoints. Increase MAX_WAYPOINTS or PATH_RESOLUTION."));
    numWaypoints = MAX_WAYPOINTS;
  }

  static Waypoint waypoints[MAX_WAYPOINTS]; // Static: lives in .bss, not stack

  // ---- Phase 1: Pre-compute all waypoints ----------------------------------
  for (int i = 0; i < numWaypoints; i++) {
    float t       = (float)(i + 1) / (float)numWaypoints;
    float interpX = current.x + dx * t;
    float interpY = current.y + dy * t;

    float r, theta;
    cartesianToPolar(interpX, interpY, r, theta);

    float extension            = constrain(r - MIN_EXTENSION, 0.0f,
                                           MAX_EXTENSION - MIN_EXTENSION);
    waypoints[i].stepperTarget = (long)(extension * STEPS_PER_MM);
    waypoints[i].servoMicros = servoAngleToMicros(theta + SERVO_ANGLE_OFFSET);
  }

  // ---- Phase 2: Scan for direction reversals, dispatch sub-moves ----------
  int subStart = 0;

  while (subStart < numWaypoints) {

    // Determine stepper direction at the start of this sub-move.
    // We use waypoints[subStart - 1] as the "previous" reference once we are
    // past the first sub-move, because current.stepperSteps is kept in sync
    // and equals waypoints[subStart - 1].stepperTarget after any prior run.
    long prevStepTarget = (subStart == 0)
                          ? current.stepperSteps
                          : waypoints[subStart - 1].stepperTarget;
    long firstDelta     = waypoints[subStart].stepperTarget - prevStepTarget;
    int  direction      = (firstDelta > 0) ?  1
                        : (firstDelta < 0) ? -1 : 0;

    // If this waypoint requires no stepper movement, sync the servo and skip.
    if (direction == 0) {
      myservo.writeMicroseconds(waypoints[subStart].servoMicros);
      subStart++;
      continue;
    }

    // Walk forward until a direction reversal is found (or end of waypoints).
    int  subEnd        = subStart;
    long subTotalSteps = abs(firstDelta);

    for (int i = subStart + 1; i < numWaypoints; i++) {
      long delta = waypoints[i].stepperTarget - waypoints[i - 1].stepperTarget;
      int  dir   = (delta > 0) ?  1
                 : (delta < 0) ? -1 : 0;

      // A non-zero direction that differs from the current run means reversal.
      // Stop here — do NOT include waypoint i in this sub-move.
      if (dir != 0 && dir != direction) break;

      subEnd = i;
      subTotalSteps += abs(delta);
    }

    // Execute this continuous-direction sub-move with its own ramp
    runSubMove(waypoints, subStart, subEnd, subTotalSteps, direction);

    subStart = subEnd + 1;
  }

  current.x = targetX;
  current.y = targetY;
}

// =============================================================================
// STEP MOTOR — Wave drive (single-phase, 4-step sequence)
// Unchanged from original. Provides lower current draw for underpowered supply.
// If more torque is needed, switch to full-step (2 pins HIGH per state).
// =============================================================================
void stepMotor(int direction) {
  static int stepIndex = 0;
  stepIndex += (direction > 0) ? 1 : -1;
  if (stepIndex > 3) stepIndex = 0;
  if (stepIndex < 0) stepIndex = 3;

  switch (stepIndex) {
    case 0: digitalWrite(STEPPER_PIN1,HIGH);digitalWrite(STEPPER_PIN2,LOW); digitalWrite(STEPPER_PIN3,LOW); digitalWrite(STEPPER_PIN4,LOW); break;
    case 1: digitalWrite(STEPPER_PIN1,LOW); digitalWrite(STEPPER_PIN2,HIGH);digitalWrite(STEPPER_PIN3,LOW); digitalWrite(STEPPER_PIN4,LOW); break;
    case 2: digitalWrite(STEPPER_PIN1,LOW); digitalWrite(STEPPER_PIN2,LOW); digitalWrite(STEPPER_PIN3,HIGH);digitalWrite(STEPPER_PIN4,LOW); break;
    case 3: digitalWrite(STEPPER_PIN1,LOW); digitalWrite(STEPPER_PIN2,LOW); digitalWrite(STEPPER_PIN3,LOW); digitalWrite(STEPPER_PIN4,HIGH);break;
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
  Serial.print(F(" mm extension (")); Serial.print(current.stepperSteps);
  Serial.println(F(" steps)"));
  Serial.print(F("Servo Angle:      ")); Serial.print(currentTheta, 2);
  Serial.println(F(" degrees"));
  Serial.println(F("------------------------\n"));
}

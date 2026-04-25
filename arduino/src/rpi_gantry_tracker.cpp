
// =============================================================================
//  POLAR GANTRY — FIRMWARE WITH RPi LINK + PREEMPTIVE MOTION
//  Target: Arduino Nano ESP32
//
//  Architecture:
//    moveTo() interpolates a straight line in Cartesian space at PATH_RESOLUTION
//    intervals. At each waypoint it writes the servo, then steps the stepper at
//    a fixed STEP_DELAY_MS until the target extension is reached.
//
//  ** PREEMPTION (added) **
//    Between waypoints — and inside the per-step loop — moveTo() checks
//    whether the Pi has sent a fresher target. If so, it aborts the
//    current motion immediately. The main loop then starts a new move
//    from the gantry's CURRENT physical position toward the new target.
//    Without this, a long move would commit the gantry for seconds while
//    the marker has already moved elsewhere — that was the source of the
//    perceived lag. With preemption the gantry is always heading toward
//    the latest target, never a stale one.
//
//  Position tracking strategy:
//    During motion, we update current.x/current.y at the END of each
//    completed waypoint. If preempted between waypoints, current.x/y is
//    already correct (last completed waypoint). If preempted mid-waypoint
//    (inside the inner step loop), we conservatively report the LAST
//    completed waypoint as our position — slightly behind reality, but
//    the next moveTo will simply close the small gap. This keeps the
//    code simple and avoids polar-to-cartesian inverse computation.
//
//  RPi communication:
//    - Serial0 on D0/D1 talks to the Raspberry Pi at 115200.
//    - Inbound  : "<X,Y>\n"        (target work coords in mm)
//    - Outbound : "<ACK:N>\n"      (after each completed OR preempted move)
//                 "<POS:X,Y>\n"    (periodic position reports)
//                 "<ERR:msg>\n"    (firmware-side error)
//    - Single-slot "pending" target. Latest write wins.
//
//  Wiring to Pi 5:
//    Pi pin 8  (GPIO14, TXD) -> Nano ESP32 D0 (RX0)
//    Pi pin 10 (GPIO15, RXD) <- Nano ESP32 D1 (TX0)
//    Pi pin 6  (GND)         -- Nano ESP32 GND
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
// D0 = RX0 (Pi link), D1 = TX0 (Pi link) — handled by Serial0, no pinMode needed.

// =============================================================================
// GANTRY CONFIGURATION
// =============================================================================
const float STEPS_PER_REV  = 2048.0f;
const float TEETH_COUNT    = 12.0f;
const float TOOTH_PITCH_MM = 3.016f;
const float STEPS_PER_MM   = STEPS_PER_REV / (TEETH_COUNT * TOOTH_PITCH_MM);
                                              // ≈ 56.59 steps/mm

const float Y_LIMIT_SWITCH_OFFSET = 48.0f;
const float MIN_EXTENSION         = Y_LIMIT_SWITCH_OFFSET;
const float MAX_EXTENSION         = 200.0f;

const float X_OFFSET = 10.0f;
const float Y_OFFSET = 124.0f;

const int   SERVO_MIN_PULSE = 500;
const int   SERVO_MAX_PULSE = 2500;
const float SERVO_MIN_DEG   = 0.0f;
const float SERVO_MAX_DEG   = 270.0f;

const float SERVO_ANGLE_OFFSET = 45.0f;
const float SERVO_MOUNT_TRIM   = 3.0f;

// 2 mm waypoints for finer preemption granularity. With preemption, more
// waypoints = more chances to react to a new target during a long move.
// The step-skipping you saw before was a per-step issue (STEP_DELAY_MS
// too low, not waypoint count), so finer waypoints are safe here.
const float PATH_RESOLUTION = 2.0f;

// =============================================================================
// STEP TIMING
// =============================================================================
const int STEP_DELAY_MS = 2;

// =============================================================================
// PI LINK CONFIG
// =============================================================================
const uint32_t PI_BAUD              = 115200;
const size_t   PI_RX_BUF_SIZE       = 2048;
const size_t   PI_FRAME_MAX         = 64;
const float    PI_WORKSPACE_RADIUS  = 67.5f;
const float    PI_WORKSPACE_SLACK   = 1.10f;
const uint32_t POS_REPORT_PERIOD_MS = 50;   // 20 Hz idle reports

// How often (in stepper steps) to check for a preempting target inside
// the inner stepping loop. Too low = wastes time checking; too high =
// laggy preemption. 32 steps ≈ 0.57 mm of arm extension.
const int     PREEMPT_CHECK_EVERY_N_STEPS = 32;

// =============================================================================
// OBJECTS & STATE
// =============================================================================
ezButton limitSwitch(LIMIT_SWITCH_PIN);
Servo    myservo;

struct State {
  float x, y;          // last reached Cartesian work position (mm)
  long  stepperSteps;  // current absolute stepper step count
} current;

struct PendingTarget {
  float x, y;
  bool  valid;
} pending = {0.0f, 0.0f, false};

static char     piRxBuf[PI_FRAME_MAX];
static size_t   piRxLen      = 0;
static bool     piInFrame    = false;
static uint32_t piCmdId      = 0;
static uint32_t lastPosReportMs = 0;

// =============================================================================
// PROTOTYPES
// =============================================================================
void homeStepper();
void ejectArm();
bool moveTo(float targetX, float targetY);   // true=completed, false=preempted
void stepMotor(int direction);
void cartesianToPolar(float x, float y, float &r, float &theta);
int  servoAngleToMicros(float angleDeg);
void printStatus();
void handleSerialInput();
void pumpPiRx();
void handlePiFrame(const char* payload);
void sendAck(uint32_t id);
void sendPos(float x, float y);
void sendErr(const char* msg);
void maybeSendPosReport();
bool pendingDiffersFrom(float x, float y);

// =============================================================================
// SETUP
// =============================================================================
void setup() {
  Serial.begin(115200);

  Serial0.setRxBufferSize(PI_RX_BUF_SIZE);
  Serial0.begin(PI_BAUD);

  limitSwitch.setDebounceTime(50);
  myservo.attach(SERVO_PIN, SERVO_MIN_PULSE, SERVO_MAX_PULSE);

  myservo.writeMicroseconds(servoAngleToMicros(90.0f + SERVO_ANGLE_OFFSET + SERVO_MOUNT_TRIM));

  pinMode(STEPPER_PIN1, OUTPUT);
  pinMode(STEPPER_PIN2, OUTPUT);
  pinMode(STEPPER_PIN3, OUTPUT);
  pinMode(STEPPER_PIN4, OUTPUT);

  Serial.println("--- GANTRY ONLINE ---");
  Serial.println("Ejecting arm...");
  ejectArm();
  homeStepper();

  current.x = -X_OFFSET;
  current.y = MIN_EXTENSION - Y_OFFSET;
  Serial.println("Moving to (0,0)...");
  moveTo(0.0f, 0.0f);
  printStatus();
  delay(1000);
  Serial.println("Entering Loop...");

  Serial0.print("<ERR:boot>\n");
}

// =============================================================================
// LOOP
// =============================================================================
void loop() {
  pumpPiRx();
  handleSerialInput();

  if (pending.valid) {
    PendingTarget cmd = pending;
    pending.valid = false;            // consumed; new frames will overwrite

    bool completed = moveTo(cmd.x, cmd.y);

    // Always report state to Pi after a motion phase (completed OR
    // preempted). The Pi treats both identically — a new ACK + POS.
    piCmdId++;
    sendAck(piCmdId);
    sendPos(current.x, current.y);

    // Note: not printing to USB Serial during fast tracking. Serial.print
    // calls have measurable overhead and would slow the loop noticeably.
    (void)completed;
  } else {
    maybeSendPosReport();
  }
}

// =============================================================================
// PI LINK — FRAMED REPLIES
// =============================================================================
void sendAck(uint32_t id) {
  Serial0.print("<ACK:");
  Serial0.print(id);
  Serial0.print(">\n");
}

void sendPos(float x, float y) {
  Serial0.print("<POS:");
  Serial0.print(x, 2);
  Serial0.print(",");
  Serial0.print(y, 2);
  Serial0.print(">\n");
  lastPosReportMs = millis();
}

void sendErr(const char* msg) {
  Serial0.print("<ERR:");
  Serial0.print(msg);
  Serial0.print(">\n");
}

void maybeSendPosReport() {
  uint32_t now = millis();
  if (now - lastPosReportMs >= POS_REPORT_PERIOD_MS) {
    sendPos(current.x, current.y);
  }
}

// =============================================================================
// PI LINK — INBOUND FRAME PARSING
// =============================================================================
void handlePiFrame(const char* payload) {
  const char* comma = strchr(payload, ',');
  if (!comma) {
    sendErr("bad_frame_no_comma");
    return;
  }

  char buf[PI_FRAME_MAX];
  strncpy(buf, payload, sizeof(buf) - 1);
  buf[sizeof(buf) - 1] = '\0';
  size_t commaIdx = comma - payload;
  buf[commaIdx] = '\0';
  char* xs = buf;
  char* ys = buf + commaIdx + 1;

  char* endX = nullptr;
  char* endY = nullptr;
  float x = strtof(xs, &endX);
  float y = strtof(ys, &endY);
  if (endX == xs || endY == ys) {
    sendErr("bad_frame_parse");
    return;
  }

  if (x * x + y * y > PI_WORKSPACE_RADIUS * PI_WORKSPACE_RADIUS * PI_WORKSPACE_SLACK) {
    sendErr("out_of_bounds");
    return;
  }

  pending.x = x;
  pending.y = y;
  pending.valid = true;
}

void pumpPiRx() {
  while (Serial0.available() > 0) {
    char c = (char)Serial0.read();

    if (c == '<') {
      piInFrame = true;
      piRxLen = 0;
      continue;
    }
    if (!piInFrame) continue;

    if (c == '>') {
      piRxBuf[piRxLen] = '\0';
      handlePiFrame(piRxBuf);
      piInFrame = false;
      piRxLen = 0;
      continue;
    }

    if (piRxLen >= PI_FRAME_MAX - 1) {
      piInFrame = false;
      piRxLen = 0;
      sendErr("frame_too_long");
      continue;
    }
    piRxBuf[piRxLen++] = c;
  }
}

// True if a new pending target exists AND it's meaningfully different
// from (x, y). Tolerance avoids preempting on identical retransmissions
// from the Pi's serial worker.
bool pendingDiffersFrom(float x, float y) {
  if (!pending.valid) return false;
  return (fabsf(pending.x - x) > 0.05f || fabsf(pending.y - y) > 0.05f);
}

// =============================================================================
// SERIAL COMMAND HANDLER (USB, non-blocking)
// =============================================================================
void handleSerialInput() {
  if (!Serial.available()) return;

  String input = Serial.readStringUntil('\n');
  input.trim();
  input.replace("\r", "");

  Serial.print(F("Received via USB: ["));
  Serial.print(input);
  Serial.println(F("]"));

  int commaIndex = input.indexOf(',');
  if (commaIndex == -1) {
    Serial.println(F("ERROR: Bad format. Use: x,y  (e.g. 40,0 or -20,35)"));
    return;
  }

  float x = input.substring(0, commaIndex).toFloat();
  float y = input.substring(commaIndex + 1).toFloat();

  if (x * x + y * y > PI_WORKSPACE_RADIUS * PI_WORKSPACE_RADIUS * PI_WORKSPACE_SLACK) {
    Serial.println(F("ERROR: target outside 135 mm workspace"));
    return;
  }

  Serial.print(F("Queued X=")); Serial.print(x, 2);
  Serial.print(F(", Y=")); Serial.println(y, 2);

  pending.x = x;
  pending.y = y;
  pending.valid = true;
}

// =============================================================================
// CARTESIAN → POLAR CONVERSION
// =============================================================================
void cartesianToPolar(float x, float y, float &r, float &theta) {
  float relX = x + X_OFFSET;
  float relY = y + Y_OFFSET;
  r     = sqrt(relX * relX + relY * relY);
  theta = atan2(relY, relX) * 180.0f / PI;
}

// =============================================================================
// SERVO ANGLE → MICROSECONDS
// =============================================================================
int servoAngleToMicros(float angleDeg) {
  float clamped = constrain(angleDeg, SERVO_MIN_DEG, SERVO_MAX_DEG);
  return (int)(SERVO_MIN_PULSE
    + (clamped - SERVO_MIN_DEG) / (SERVO_MAX_DEG - SERVO_MIN_DEG)
    * (float)(SERVO_MAX_PULSE - SERVO_MIN_PULSE));
}

// =============================================================================
// MOVETO  (preemptive)
//
// Returns true if completed normally, false if preempted by a fresher
// pending target. In either case `current.x/y` reflect a valid (last
// reached) physical position when the function returns.
//
// Preemption checks happen at two granularities:
//   1. Between waypoints (fast, frequent)
//   2. Inside the inner step loop, every PREEMPT_CHECK_EVERY_N_STEPS
//      steps (so even a single long-extension waypoint can be aborted)
// =============================================================================
bool moveTo(float targetX, float targetY) {
  const float startX = current.x;
  const float startY = current.y;
  const float dx     = targetX - startX;
  const float dy     = targetY - startY;
  const float distance = sqrt(dx * dx + dy * dy);
  if (distance < 0.001f) return true;

  const int numWaypoints = (int)ceil(distance / PATH_RESOLUTION);

  for (int i = 1; i <= numWaypoints; i++) {
    pumpPiRx();
    maybeSendPosReport();

    // --- Preemption check between waypoints ---
    if (pendingDiffersFrom(targetX, targetY)) {
      // We have completed (i-1)/numWaypoints of the line. current.x/y
      // was last updated at the end of the previous iteration, so it's
      // already correct.
      return false;
    }

    // Compute this waypoint's position from the FUNCTION-ENTRY origin
    // (startX, startY). This avoids accumulating floating-point drift.
    const float t = (float)i / (float)numWaypoints;
    const float interpX = startX + dx * t;
    const float interpY = startY + dy * t;

    // --- Servo angle ---
    float r, theta;
    cartesianToPolar(interpX, interpY, r, theta);
    const float angle = theta + SERVO_ANGLE_OFFSET + SERVO_MOUNT_TRIM;
    myservo.writeMicroseconds(servoAngleToMicros(angle));

    // --- Step extension toward this waypoint ---
    const float extension = constrain(r - MIN_EXTENSION, 0.0f, MAX_EXTENSION - MIN_EXTENSION);
    const long  targetSteps = (long)(extension * STEPS_PER_MM);

    int stepsSinceCheck = 0;
    while (current.stepperSteps != targetSteps) {
      const int direction = (current.stepperSteps < targetSteps) ? 1 : -1;
      stepMotor(direction);
      current.stepperSteps += direction;
      delay(STEP_DELAY_MS);

      // --- Preemption check inside the step loop ---
      if (++stepsSinceCheck >= PREEMPT_CHECK_EVERY_N_STEPS) {
        stepsSinceCheck = 0;
        pumpPiRx();
        if (pendingDiffersFrom(targetX, targetY)) {
          // Mid-waypoint preempt: report the LAST completed waypoint as
          // current position. The next moveTo will close any small gap.
          const float t_done = (float)(i - 1) / (float)numWaypoints;
          current.x = startX + dx * t_done;
          current.y = startY + dy * t_done;
          return false;
        }
      }
    }

    // Successfully reached this waypoint. Commit position.
    current.x = interpX;
    current.y = interpY;
  }

  // Final endpoint snap (eliminates any 1-step rounding drift).
  current.x = targetX;
  current.y = targetY;
  return true;
}

// =============================================================================
// STEP MOTOR
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
// HOME STEPPER
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
// EJECT ARM
// =============================================================================
void ejectArm() {
  Serial.println("Ejecting...");

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

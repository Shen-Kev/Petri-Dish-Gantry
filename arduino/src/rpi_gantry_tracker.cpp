// =============================================================================
//  POLAR GANTRY — FIRMWARE WITH RPi LINK
//  Target: Arduino Nano ESP32
//
//  Architecture:
//    moveTo() interpolates a straight line in Cartesian space at PATH_RESOLUTION
//    intervals. At each waypoint it writes the servo, then steps the stepper at
//    a fixed STEP_DELAY_MS until the target extension is reached. No buffering,
//    no ramping, no lookahead — structurally identical to the working test loop.
//
//  RPi communication (added):
//    - Serial0 on D0/D1 talks to the Raspberry Pi at 115200.
//    - Inbound  : "<X,Y>\n"        (target work coords in mm)
//    - Outbound : "<ACK:N>\n"      (after each completed move)
//                 "<POS:X,Y>\n"    (periodic position reports)
//                 "<ERR:msg>\n"    (firmware-side error)
//    - "Drop-old, keep-latest": only the most recent pending target is
//      ever executed. While moveTo is running, new frames overwrite the
//      pending target rather than queuing — matches the Pi's size-1 queue.
//    - Serial0 RX buffer is enlarged to 2 KB so commands sent during a
//      long move don't overflow.
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

const float Y_LIMIT_SWITCH_OFFSET = 48.0f;   // mm, pivot → end effector at limit switch
const float MIN_EXTENSION         = Y_LIMIT_SWITCH_OFFSET;
const float MAX_EXTENSION         = 200.0f;  // mm, maximum physical reach

//const float X_OFFSET       = 8.0f;           // mm, pivot → work origin
const float X_OFFSET = 15.0f;
const float Y_OFFSET       = 120.0f;         // mm, pivot → work origin

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

const float PATH_RESOLUTION = 5.0f;          // mm per interpolation waypoint

// =============================================================================
// STEP TIMING
// =============================================================================
const int STEP_DELAY_MS = 2;                 // ms per stepper step. 2 seems to be the limit

// =============================================================================
// PI LINK CONFIG
// =============================================================================
const uint32_t PI_BAUD             = 115200;
const size_t   PI_RX_BUF_SIZE      = 2048;   // ESP32 RX buffer (default 256 too small)
const size_t   PI_FRAME_MAX        = 64;     // max chars between < and >
const float    PI_WORKSPACE_RADIUS = 67.5f;  // 135 mm diameter circle
const float    PI_WORKSPACE_SLACK  = 1.10f;  // 10% slack for rounding/noise
const uint32_t POS_REPORT_PERIOD_MS = 100;   // 10 Hz position reports during moves

// =============================================================================
// OBJECTS & STATE
// =============================================================================
ezButton limitSwitch(LIMIT_SWITCH_PIN);
Servo    myservo;

struct State {
  float x, y;         // Current Cartesian work position (mm)
  long  stepperSteps; // Current absolute stepper step count
} current;

// Pi protocol state
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
void moveTo(float targetX, float targetY);
void stepMotor(int direction);
void cartesianToPolar(float x, float y, float &r, float &theta);
int  servoAngleToMicros(float angleDeg);
void printStatus();
void testPlus();
void testDiamond();
void handleSerialInput();
void pumpPiRx();
void handlePiFrame(const char* payload);
void sendAck(uint32_t id);
void sendPos(float x, float y);
void sendErr(const char* msg);
void maybeSendPosReport();

// =============================================================================
// SETUP
// =============================================================================
void setup() {
  Serial.begin(115200);                        // USB debug

  // Enlarge Serial0 RX buffer BEFORE begin() so commands sent during a
  // long move don't overflow. 2 KB ≈ 130 frames at 15 bytes each.
  Serial0.setRxBufferSize(PI_RX_BUF_SIZE);
  Serial0.begin(PI_BAUD);                      // Pi link on D0/D1

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

  // Tell the Pi we're ready (framed so the Pi parser consumes it cleanly).
  Serial0.print("<ERR:boot>\n");
}

// =============================================================================
// LOOP
//
// Pi commands are the primary path. handleSerialInput() is left as a
// non-blocking fallback for manual testing via USB Serial Monitor.
// =============================================================================
void loop() {
  // 1. Drain Pi RX into `pending` (latest target wins).
  pumpPiRx();

  // 2. Also accept manual commands typed in USB Serial Monitor.
  handleSerialInput();

  // 3. If we have a fresh target, execute it.
  if (pending.valid) {
    PendingTarget cmd = pending;
    pending.valid = false;            // consumed; new frames will overwrite it

    moveTo(cmd.x, cmd.y);
    piCmdId++;
    sendAck(piCmdId);
    sendPos(current.x, current.y);

    Serial.print("[Pi cmd "); Serial.print(piCmdId);
    Serial.print("] X="); Serial.print(cmd.x, 2);
    Serial.print(" Y="); Serial.println(cmd.y, 2);
  } else {
    // 4. Idle position reports so the Pi's HUD stays fresh.
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
//
// Byte-by-byte state machine. Tolerates garbage outside frames, partial
// frames spanning loop iterations, and overlong frames (resets cleanly).
// Updates `pending` with the latest target — does NOT call moveTo here,
// because the loop owns motion.
// =============================================================================
void handlePiFrame(const char* payload) {
  // Expect "X,Y" — two floats separated by a comma.
  const char* comma = strchr(payload, ',');
  if (!comma) {
    sendErr("bad_frame_no_comma");
    return;
  }

  // Split into two C-strings in a local buffer.
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

  // Soft sanity clamp for the 135 mm circular workspace. The Pi already
  // gates this, but a second check on the firmware side is cheap insurance
  // against garbled frames commanding wild moves.
  if (x * x + y * y > PI_WORKSPACE_RADIUS * PI_WORKSPACE_RADIUS * PI_WORKSPACE_SLACK) {
    sendErr("out_of_bounds");
    return;
  }

  // Update pending target. The main loop will pick this up after the
  // current moveTo (if any) returns.
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
    if (!piInFrame) continue;       // ignore noise between frames

    if (c == '>') {
      piRxBuf[piRxLen] = '\0';
      handlePiFrame(piRxBuf);
      piInFrame = false;
      piRxLen = 0;
      continue;
    }

    if (piRxLen >= PI_FRAME_MAX - 1) {  // overflow — drop frame
      piInFrame = false;
      piRxLen = 0;
      sendErr("frame_too_long");
      continue;
    }
    piRxBuf[piRxLen++] = c;
  }
}

// =============================================================================
// SERIAL COMMAND HANDLER (USB, non-blocking)
//
// Originally blocked with `while (!Serial.available())`. Made non-blocking
// so the Pi link runs in parallel. Type "x,y" in Serial Monitor and press
// Enter for manual testing. Manual commands also go through `pending` so
// they get the same workspace clamp as Pi commands.
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

  // Same workspace clamp the Pi link uses.
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
// MODIFIED: pumps the Pi RX buffer between waypoints so commands sent
// during the move don't overflow Serial0's RX buffer, and emits periodic
// position reports so the Pi's HUD stays live during long moves.
// =============================================================================
void moveTo(float targetX, float targetY) {
  float dx       = targetX - current.x;
  float dy       = targetY - current.y;
  float distance = sqrt(dx * dx + dy * dy);
  if (distance < 0.001f) return;

  int numWaypoints = (int)ceil(distance / PATH_RESOLUTION);

  for (int i = 1; i <= numWaypoints; i++) {
    // Keep the Pi RX buffer drained during long moves so we don't
    // overflow. Frames received here update `pending` for AFTER this
    // move completes — they don't preempt the current motion.
    pumpPiRx();
    maybeSendPosReport();

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

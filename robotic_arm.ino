#include <Servo.h>

/*
  2-DOF Planar Arm (L1 = 10 cm, L2 = 10 cm)
  -------------------------------------------------
  - Shoulder servo: MG996R on pin 9
  - Elbow servo:    MG90S on pin 10
  - Joystick X: A0, Y: A1

  Control strategy:
  1) Joystick commands end-effector velocity in Cartesian space (x, y),
     not direct servo angles.
  2) Inverse kinematics converts (x, y) -> (shoulder, elbow angles).
  3) Movement is updated at a fixed interval using millis() (non-blocking).
  4) Joystick input is low-pass filtered and has deadband to reduce noise.
  5) Workspace and servo limits are enforced to prevent invalid commands.
*/

Servo shoulderServo;
Servo elbowServo;

// ---------------- Hardware pins ----------------
const uint8_t SHOULDER_PIN = 9;
const uint8_t ELBOW_PIN = 10;
const uint8_t JOY_X_PIN = A0;
const uint8_t JOY_Y_PIN = A1;

// ---------------- Arm geometry (cm) ----------------
const float L1 = 10.0f;
const float L2 = 10.0f;

// ---------------- Servo angle limits (degrees) ----------------
const float SHOULDER_MIN_DEG = 0.0f;
const float SHOULDER_MAX_DEG = 180.0f;
const float ELBOW_MIN_DEG = 0.0f;
const float ELBOW_MAX_DEG = 180.0f;

// These offsets convert mathematical IK angles to your mechanical servo zero.
// Tune once if your physical "zero" differs.
const float SHOULDER_ZERO_OFFSET_DEG = 90.0f;
const float ELBOW_ZERO_OFFSET_DEG = 0.0f;

// ---------------- Control timing ----------------
const uint16_t CONTROL_PERIOD_MS = 20; // ~50 Hz updates (smooth for hobby servos)
unsigned long lastControlMs = 0;

// ---------------- Joystick filtering / shaping ----------------
const float JOY_ALPHA = 0.20f;          // low-pass filter strength (0..1)
const float JOY_DEADBAND = 0.06f;       // remove small stick noise around center
float joyXFiltered = 0.0f;
float joyYFiltered = 0.0f;

// ---------------- Cartesian motion tuning ----------------
const float MAX_SPEED_CM_S = 6.0f;      // max end-effector speed from joystick

// ---------------- Servo smoothing / anti-jitter ----------------
const float MAX_SERVO_STEP_DEG = 2.0f;  // max change per control tick per joint
const float WRITE_EPS_DEG = 0.3f;       // do not rewrite tiny changes

// Desired (commanded) end-effector position (cm)
float targetX = L1 + L2 - 2.0f; // start near full extension but safely inside boundary
float targetY = 0.0f;

// Current commanded servo angles (deg)
float shoulderCmdDeg = SHOULDER_ZERO_OFFSET_DEG;
float elbowCmdDeg = ELBOW_ZERO_OFFSET_DEG;

// ---------------- Utility helpers ----------------
float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

float applyDeadband(float v, float deadband) {
  if (fabs(v) < deadband) return 0.0f;

  // Re-scale so output remains continuous after deadband.
  if (v > 0.0f) return (v - deadband) / (1.0f - deadband);
  return (v + deadband) / (1.0f - deadband);
}

float moveToward(float current, float target, float maxStep) {
  float delta = target - current;
  if (fabs(delta) <= maxStep) return target;
  return current + (delta > 0.0f ? maxStep : -maxStep);
}

// Keep target point in reachable annulus for a 2-link arm.
// Reachable radius: |L1-L2| <= r <= (L1+L2)
void enforceWorkspace(float &x, float &y) {
  float r = sqrt(x * x + y * y);
  const float rMin = fabs(L1 - L2) + 0.5f; // +margin avoids singularity at full fold
  const float rMax = (L1 + L2) - 0.5f;     // -margin avoids singularity at full stretch

  if (r < 1e-4f) {
    x = rMin;
    y = 0.0f;
    return;
  }

  if (r < rMin) {
    float s = rMin / r;
    x *= s;
    y *= s;
  } else if (r > rMax) {
    float s = rMax / r;
    x *= s;
    y *= s;
  }
}

/*
  Inverse Kinematics for planar 2-link arm
  ----------------------------------------
  Given end-effector target (x, y):

  1) Compute elbow angle using law of cosines:
     cos(theta2) = (x^2 + y^2 - L1^2 - L2^2) / (2 L1 L2)

  2) Select elbow configuration:
     theta2 = acos(cos(theta2))  (elbow-down branch)

  3) Compute shoulder angle:
     theta1 = atan2(y, x) - atan2(L2*sin(theta2), L1 + L2*cos(theta2))

  Returns true if a valid solution exists.
*/
bool solveIK(float x, float y, float &shoulderDegOut, float &elbowDegOut) {
  float c2 = (x * x + y * y - L1 * L1 - L2 * L2) / (2.0f * L1 * L2);

  // Numerical safety clamp.
  c2 = clampf(c2, -1.0f, 1.0f);

  // Elbow-down solution branch.
  float theta2 = acos(c2);

  float k1 = L1 + L2 * cos(theta2);
  float k2 = L2 * sin(theta2);
  float theta1 = atan2(y, x) - atan2(k2, k1);

  float shoulderDeg = degrees(theta1) + SHOULDER_ZERO_OFFSET_DEG;
  float elbowDeg = degrees(theta2) + ELBOW_ZERO_OFFSET_DEG;

  shoulderDegOut = shoulderDeg;
  elbowDegOut = elbowDeg;

  // Solution exists mathematically, but still enforce servo limits.
  if (shoulderDegOut < SHOULDER_MIN_DEG || shoulderDegOut > SHOULDER_MAX_DEG) return false;
  if (elbowDegOut < ELBOW_MIN_DEG || elbowDegOut > ELBOW_MAX_DEG) return false;

  return true;
}

void setup() {
  Serial.begin(115200);

  shoulderServo.attach(SHOULDER_PIN);
  elbowServo.attach(ELBOW_PIN);

  // Start from current target by solving IK once.
  float sDeg = SHOULDER_ZERO_OFFSET_DEG;
  float eDeg = ELBOW_ZERO_OFFSET_DEG;
  enforceWorkspace(targetX, targetY);
  if (solveIK(targetX, targetY, sDeg, eDeg)) {
    shoulderCmdDeg = sDeg;
    elbowCmdDeg = eDeg;
  }

  shoulderServo.write((int)round(clampf(shoulderCmdDeg, SHOULDER_MIN_DEG, SHOULDER_MAX_DEG)));
  elbowServo.write((int)round(clampf(elbowCmdDeg, ELBOW_MIN_DEG, ELBOW_MAX_DEG)));

  Serial.println(F("2-DOF joystick IK control ready."));
  lastControlMs = millis();
}

void loop() {
  unsigned long now = millis();
  if (now - lastControlMs < CONTROL_PERIOD_MS) {
    return; // non-blocking: do nothing until next control tick
  }

  float dt = (now - lastControlMs) / 1000.0f;
  lastControlMs = now;

  // 1) Read joystick and normalize to [-1, +1]
  int rawX = analogRead(JOY_X_PIN);
  int rawY = analogRead(JOY_Y_PIN);

  float joyX = ((float)rawX - 512.0f) / 512.0f;
  float joyY = ((float)rawY - 512.0f) / 512.0f;

  joyX = clampf(joyX, -1.0f, 1.0f);
  joyY = clampf(joyY, -1.0f, 1.0f);

  // 2) Low-pass filter to reduce ADC noise
  joyXFiltered += JOY_ALPHA * (joyX - joyXFiltered);
  joyYFiltered += JOY_ALPHA * (joyY - joyYFiltered);

  // 3) Deadband to prevent drift around center
  float joyXCmd = applyDeadband(joyXFiltered, JOY_DEADBAND);
  float joyYCmd = applyDeadband(joyYFiltered, JOY_DEADBAND);

  // 4) Integrate joystick command as Cartesian velocity
  float vx = joyXCmd * MAX_SPEED_CM_S;
  float vy = joyYCmd * MAX_SPEED_CM_S;
  targetX += vx * dt;
  targetY += vy * dt;

  // 5) Keep target inside reachable workspace
  enforceWorkspace(targetX, targetY);

  // 6) IK solve for target angles
  float shoulderTargetDeg = shoulderCmdDeg;
  float elbowTargetDeg = elbowCmdDeg;
  bool ok = solveIK(targetX, targetY, shoulderTargetDeg, elbowTargetDeg);

  if (ok) {
    // 7) Smooth angle command to avoid jumps/jitter
    shoulderCmdDeg = moveToward(shoulderCmdDeg, shoulderTargetDeg, MAX_SERVO_STEP_DEG);
    elbowCmdDeg = moveToward(elbowCmdDeg, elbowTargetDeg, MAX_SERVO_STEP_DEG);

    shoulderCmdDeg = clampf(shoulderCmdDeg, SHOULDER_MIN_DEG, SHOULDER_MAX_DEG);
    elbowCmdDeg = clampf(elbowCmdDeg, ELBOW_MIN_DEG, ELBOW_MAX_DEG);

    static float lastWrittenShoulder = -1000.0f;
    static float lastWrittenElbow = -1000.0f;

    if (fabs(shoulderCmdDeg - lastWrittenShoulder) > WRITE_EPS_DEG) {
      shoulderServo.write((int)round(shoulderCmdDeg));
      lastWrittenShoulder = shoulderCmdDeg;
    }

    if (fabs(elbowCmdDeg - lastWrittenElbow) > WRITE_EPS_DEG) {
      elbowServo.write((int)round(elbowCmdDeg));
      lastWrittenElbow = elbowCmdDeg;
    }
  }

  // Optional lightweight telemetry at ~10 Hz
  static uint8_t debugDivider = 0;
  debugDivider++;
  if (debugDivider >= 5) {
    debugDivider = 0;
    Serial.print(F("X:")); Serial.print(targetX, 2);
    Serial.print(F(" Y:")); Serial.print(targetY, 2);
    Serial.print(F(" S:")); Serial.print(shoulderCmdDeg, 1);
    Serial.print(F(" E:")); Serial.println(elbowCmdDeg, 1);
  }
}

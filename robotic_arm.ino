#include <Servo.h>

/*
  2-DOF Planar Arm (L1 = 10 cm, L2 = 10 cm)
  -------------------------------------------------
  - Shoulder servo: MG996R on pin 9
  - Elbow servo:    MG90S on pin 10

  Control strategy:
  1) End effector is commanded automatically along Cartesian x.
     Cartesian y is held at a fixed safe value.
  2) Inverse kinematics converts (x, y) -> (shoulder, elbow angles).
  3) Movement is updated at a fixed interval using millis() (non-blocking).
  4) Workspace and servo limits are enforced to prevent invalid commands.
  5) Angle interpolation + slew limits keep motion smooth.
*/

Servo shoulderServo;
Servo elbowServo;

// ---------------- Hardware pins ----------------
const uint8_t SHOULDER_PIN = 9;
const uint8_t ELBOW_PIN = 10;

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

// ---------------- Cartesian motion tuning ----------------
const float FIXED_Y_CM = 10.0f;         // keep Y fixed at a safe value
const float X_MIN_CM = -8.0f;           // safe X sweep range, inside reachable workspace
const float X_MAX_CM = 8.0f;
const float X_SWEEP_SPEED_CM_S = 1.5f;  // slow auto motion along X
int8_t xDirection = 1;

// ---------------- Servo smoothing / anti-jitter ----------------
const float MAX_SHOULDER_STEP_DEG = 1.1f; // MG996R: tighter per-tick slew limit
const float MAX_ELBOW_STEP_DEG = 1.8f;    // lighter elbow can move a bit faster
const float ANGLE_INTERP_RATE = 10.0f;    // interpolation speed (1/s) toward IK target
const float WRITE_EPS_DEG = 0.35f;        // do not rewrite tiny changes

// Desired (commanded) end-effector position (cm)
float targetX = 0.0f;
float targetY = FIXED_Y_CM;

// Current commanded servo angles (deg)
float shoulderCmdDeg = SHOULDER_ZERO_OFFSET_DEG;
float elbowCmdDeg = ELBOW_ZERO_OFFSET_DEG;
float shoulderInterpDeg = SHOULDER_ZERO_OFFSET_DEG;
float elbowInterpDeg = ELBOW_ZERO_OFFSET_DEG;

// ---------------- Utility helpers ----------------
float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

float moveToward(float current, float target, float maxStep) {
  float delta = target - current;
  if (fabs(delta) <= maxStep) return target;
  return current + (delta > 0.0f ? maxStep : -maxStep);
}

float toElbowServoDeg(float elbowIkDeg) {
  // Flip elbow servo direction at final write stage only.
  return clampf(180.0f - elbowIkDeg, ELBOW_MIN_DEG, ELBOW_MAX_DEG);
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
    shoulderInterpDeg = sDeg;
    elbowInterpDeg = eDeg;
  }

  shoulderServo.write((int)round(clampf(shoulderCmdDeg, SHOULDER_MIN_DEG, SHOULDER_MAX_DEG)));
  elbowServo.write((int)round(toElbowServoDeg(elbowCmdDeg)));

  Serial.println(F("2-DOF X-axis auto-sweep IK control ready."));
  lastControlMs = millis();
}

void loop() {
  unsigned long now = millis();
  if (now - lastControlMs < CONTROL_PERIOD_MS) {
    return; // non-blocking: do nothing until next control tick
  }

  float dt = (now - lastControlMs) / 1000.0f;
  lastControlMs = now;

  // For robust filtering and interpolation in case of occasional loop jitter.
  dt = clampf(dt, 0.001f, 0.10f);

  // 1) Auto-sweep X back and forth; hold Y constant.
  targetY = FIXED_Y_CM;
  targetX += xDirection * X_SWEEP_SPEED_CM_S * dt;

  if (targetX >= X_MAX_CM) {
    targetX = X_MAX_CM;
    xDirection = -1;
  } else if (targetX <= X_MIN_CM) {
    targetX = X_MIN_CM;
    xDirection = 1;
  }

  // 2) Workspace safety clamp for the full (x, y) point.
  enforceWorkspace(targetX, targetY);

  // Keep Y fixed after workspace enforcement while still bounding X to requested range.
  targetY = FIXED_Y_CM;
  targetX = clampf(targetX, X_MIN_CM, X_MAX_CM);

  // 3) IK solve for target angles
  float shoulderTargetDeg = shoulderCmdDeg;
  float elbowTargetDeg = elbowCmdDeg;
  bool ok = solveIK(targetX, targetY, shoulderTargetDeg, elbowTargetDeg);

  if (ok) {
    // 4) Interpolate IK target angles first, then apply per-joint slew limits.
    float interpAlpha = clampf(ANGLE_INTERP_RATE * dt, 0.0f, 1.0f);
    shoulderInterpDeg += interpAlpha * (shoulderTargetDeg - shoulderInterpDeg);
    elbowInterpDeg += interpAlpha * (elbowTargetDeg - elbowInterpDeg);

    shoulderCmdDeg = moveToward(shoulderCmdDeg, shoulderInterpDeg, MAX_SHOULDER_STEP_DEG);
    elbowCmdDeg = moveToward(elbowCmdDeg, elbowInterpDeg, MAX_ELBOW_STEP_DEG);

    shoulderCmdDeg = clampf(shoulderCmdDeg, SHOULDER_MIN_DEG, SHOULDER_MAX_DEG);
    elbowCmdDeg = clampf(elbowCmdDeg, ELBOW_MIN_DEG, ELBOW_MAX_DEG);

    static float lastWrittenShoulder = -1000.0f;
    static float lastWrittenElbow = -1000.0f;

    if (fabs(shoulderCmdDeg - lastWrittenShoulder) > WRITE_EPS_DEG) {
      shoulderServo.write((int)round(shoulderCmdDeg));
      lastWrittenShoulder = shoulderCmdDeg;
    }

    if (fabs(elbowCmdDeg - lastWrittenElbow) > WRITE_EPS_DEG) {
      float elbowServoDeg = toElbowServoDeg(elbowCmdDeg);
      elbowServo.write((int)round(elbowServoDeg));
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

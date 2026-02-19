#include <Servo.h>

Servo shoulder;
Servo elbow;

const float L1 = 10.0;
const float L2 = 10.0;

const int SHOULDER_STRAIGHT = 94;
const int ELBOW_STRAIGHT = 92;

int currentShoulder = SHOULDER_STRAIGHT;
int currentElbow = ELBOW_STRAIGHT;

void moveSmooth(Servo &s, int &currentPos, int targetPos) {
  while (currentPos != targetPos) {
    if (currentPos < targetPos) currentPos++;
    else currentPos--;

    s.write(currentPos);
    delay(10);
  }
}

void setup() {
  Serial.begin(9600);

  shoulder.attach(9);
  elbow.attach(11);

  shoulder.write(currentShoulder);
  elbow.write(currentElbow);

  Serial.println("Enter X and Y:");
}

void loop() {

  if (Serial.available()) {

    float x = Serial.parseFloat();
    float y = Serial.parseFloat();

    float r = sqrt(x*x + y*y);

    if (r > (L1 + L2) || r < abs(L1 - L2)) {
      Serial.println("Point not reachable.");
      return;
    }

    // ---- Inverse Kinematics ----
    float cosT2 = (x*x + y*y - L1*L1 - L2*L2) / (2 * L1 * L2);
    cosT2 = constrain(cosT2, -1.0, 1.0);

    float t2 = acos(cosT2);

    float k1 = L1 + L2 * cos(t2);
    float k2 = L2 * sin(t2);

    float t1 = atan2(y, x) - atan2(k2, k1);

    int targetShoulder = SHOULDER_STRAIGHT + degrees(t1);
    int targetElbow = ELBOW_STRAIGHT + degrees(t2);   // FIXED

    targetShoulder = constrain(targetShoulder, 0, 180);
    targetElbow = constrain(targetElbow, 0, 180);

    moveSmooth(shoulder, currentShoulder, targetShoulder);
    moveSmooth(elbow, currentElbow, targetElbow);

    Serial.println("Position reached.");
  }
}

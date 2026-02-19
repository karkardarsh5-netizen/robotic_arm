#include <Servo.h>

Servo base;
Servo elbow;

int xpin = A0;
int ypin = A1;

int deadzoneLow = 480;
int deadzoneHigh = 550;

int baseInitial = 94;
int elbowInitial = 92;

int baseCurrent = 90;
int elbowCurrent = 90;

int baseTarget = 90;
int elbowTarget = 90;

void setup() {
  base.attach(9);
  elbow.attach(11);

  base.write(baseInitial);
  elbow.write(elbowInitial);
}

void loop() {

  int xin = analogRead(xpin);
  int yin = analogRead(ypin);

  bool xInDeadzone = (xin >= deadzoneLow && xin <= deadzoneHigh);
  bool yInDeadzone = (yin >= deadzoneLow && yin <= deadzoneHigh);

  // If both axes are in deadzone → return to initial
  if (xInDeadzone && yInDeadzone) {
    baseTarget = baseInitial;
    elbowTarget = elbowInitial;
  }
  else {
    // BASE (reversed direction)
    if (!xInDeadzone) {
      baseTarget = map(xin, 0, 1023, 180, 0);
    }

    // ELBOW (normal direction)
    if (!yInDeadzone) {
      elbowTarget = map(yin, 0, 1023, 0, 180);
    }
  }

  // Smooth movement
  if (baseCurrent < baseTarget) baseCurrent++;
  else if (baseCurrent > baseTarget) baseCurrent--;

  if (elbowCurrent < elbowTarget) elbowCurrent++;
  else if (elbowCurrent > elbowTarget) elbowCurrent--;

  base.write(baseCurrent);
  elbow.write(elbowCurrent);

  delay(10);  // smaller = faster, bigger = smoother/slower
}

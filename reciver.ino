#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Servo.h>

RF24 radio(9, 10);
const byte address[6] = "00001";

Servo base;
Servo elbow;

struct Data {
  int x;
  int y;
};

Data data;

int baseCurrent = 94;
int elbowCurrent = 100;

int baseTarget = 90;
int elbowTarget = 90;

void setup() {

  base.attach(5);
  elbow.attach(6);

  base.write(94);
  elbow.write(92);
  delay(500);   // stabilize servos

  radio.begin();
  radio.setPALevel(RF24_PA_LOW);
  radio.setDataRate(RF24_250KBPS);   // more stable
  radio.openReadingPipe(0, address);
  radio.startListening();
}

void loop() {

  if (radio.available()) {
    radio.read(&data, sizeof(Data));

    baseTarget  = map(data.x, 0, 1023, 180, 0);   // reversed
    elbowTarget = map(data.y, 0, 1023, 0, 180);
  }

  // Smooth stepping
  if (baseCurrent < baseTarget) baseCurrent++;
  else if (baseCurrent > baseTarget) baseCurrent--;

  if (elbowCurrent < elbowTarget) elbowCurrent++;
  else if (elbowCurrent > elbowTarget) elbowCurrent--;

  base.write(baseCurrent);
  elbow.write(elbowCurrent);

  delay(8);
}

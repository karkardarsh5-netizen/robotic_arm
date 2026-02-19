#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(9, 10);   // CE, CSN
const byte address[6] = "00001";

struct Data {
  int x;
  int y;
};

Data data;

void setup() {
  radio.begin();
  radio.setPALevel(RF24_PA_LOW);
  radio.setDataRate(RF24_250KBPS);   // more stable
  radio.openWritingPipe(address);
  radio.stopListening();
}

void loop() {

  data.x = analogRead(A0);
  data.y = analogRead(A1);

  radio.write(&data, sizeof(Data));

  delay(20);
}

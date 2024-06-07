#include <SoftwareSerial.h>
#include "TFMini.h"

//SoftwareSerial pin setup for TFMini sensor
SoftwareSerial SerialTFMini(2, 3); // RX, TX

TFMini tfmini;  // TFMini sensor object

//To get data from TFMini sensor
void getTFminiData(int* distance, int* strength) {
  static char i = 0;
  char j = 0;
  int checksum = 0;
  static int rx[9];

  if (SerialTFMini.available()) {
    rx[i] = SerialTFMini.read();
    if (rx[0] != 0x59) {
      i = 0;
    } 
    else if (i == 1 && rx[1] != 0x59) {
      i = 0;
    } 
    else if (i == 8) {
      for (j = 0; j < 8; j++) {
        checksum += rx[j];
      }
      if (rx[8] == (checksum % 256)) {
        *distance = rx[2] + rx[3] * 256;
        *strength = rx[4] + rx[5] * 256;
      }
      i = 0;
    } 
    else {
      i++;
    }
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial);  //Need for USB port. Wait until connection is true
  Serial.println("Initializing...");
  SerialTFMini.begin(115200);  //Start SoftwareSerial port for TFMini
  tfmini.begin(&SerialTFMini);  //Start TFMini sensor
}

void loop() {
  int distance = 0;
  int strength = 0;

  getTFminiData(&distance, &strength);

  while (!distance) {
    getTFminiData(&distance, &strength);
    
    if (distance) {
      Serial.print(distance);
      Serial.print("cm\t");
      Serial.print("strength: ");
      Serial.println(strength);
    }
  }
  delay(100);
}

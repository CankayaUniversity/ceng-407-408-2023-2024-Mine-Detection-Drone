#include <Servo.h>  
#include <SoftwareSerial.h>
#include <SPI.h>
#include "TFMini.h"   //We used this library for Lidar sensor.
//We used these two libraries to NRF24L01 (communication).
#include <nRF24L01.h>   
#include <RF24.h>

Servo myservo;
SoftwareSerial SerialTFMini(2, 3);  // RX, TX
TFMini tfmini;

#define CE_PIN 9
#define CSN_PIN 10

RF24 radio(CE_PIN, CSN_PIN);
const byte address[][6] = { "00001", "00002" };


int pos = 0;  
int distance = 0;
int adcVal = 0;
int adcVal2 = 0;
float reference = 0;
float val = 0;

//0 - During the takeoff
//1 - During the flight.
//2 - During the landing.
int state = 0;
char msg[32];

void setup() {
  Serial.begin(115200);
  radio.begin();
  radio.openWritingPipe(address[1]);
  radio.openReadingPipe(0, address[0]);

  radio.setPALevel(RF24_PA_HIGH);
  radio.setDataRate(RF24_2MBPS);
  
  myservo.attach(8);   //We connect the servo motor to the pin 8 on arduino uno(transmitter).
  SerialTFMini.begin(115200);   // Start SoftwareSerial port for TFMini
  tfmini.begin(&SerialTFMini);  // Start TFMini sensor
  radio.setPALevel(RF24_PA_HIGH);
}
void getTFminiData(int* distance) {
  static char i = 0;
  char j = 0;
  int checksum = 0;
  static int rx[9];
  //Distance sensor (Lidar) code.
  while (SerialTFMini.available()) {
    rx[i] = SerialTFMini.read();
    if (rx[0] != 0x59) {
      i = 0;
    } else if (i == 1 && rx[1] != 0x59) {
      i = 0;
    } else if (i == 8) {
      for (j = 0; j < 8; j++) {
        checksum += rx[j];
      }
      if (rx[8] == (checksum % 256)) {
        *distance = rx[2] + rx[3] * 256;
      }
      i = 0;
    } else {
      i++;
    }
  }
  delay(100);
}

void CheckState() {
  delay(5);
  radio.startListening();
  delay(100);
  if (radio.available() || radio.available() || radio.available() || radio.available()) {
    Serial.println("MSG:changestate:");

    radio.read(&msg, 32);
    String str(msg);
    Serial.println("MSG:changestate:" + String(state) + ".");

    str.replace("\n", "");
    str.replace("\r", "");
    String key, value;
    if (str.indexOf(":") > 0) {
      key = str.substring(0, str.indexOf(":"));
      value = str.substring(str.indexOf(":") + 1);
    } else {
      key = str;
      value = 1;
    }
    if (key == "changestate") {
      state = value.toInt();
      Serial.println("MSG:changestate:" + String(state) + ".");
    }
    radio.stopListening();
    delay(5);
    radio.write(&msg, sizeof(msg));
  }
  delay(5);
  radio.stopListening();
  delay(5);
}

void loop() {
  CheckState();


  if (state == 0) {
    myservo.detach();
    delay(100);
  } else if (state == 1) {
    adcVal = analogRead(0);
    adcVal2 = analogRead(1);
    val = (adcVal2 + adcVal) / 2;
    getTFminiData(&distance);
    float dataToSend[2] = { val, (float)distance };
    radio.write(&dataToSend, sizeof(dataToSend));
    if (distance > 50) {
      myservo.attach(8);
      myservo.write(0);
      getTFminiData(&distance);

    } else if (distance <= 50 && distance >= 30) {

      myservo.detach();   //To stop the servo motor.

    } else {
      myservo.attach(8);
      myservo.write(180);
      getTFminiData(&distance);
    }
    Serial.println("Distance: " +String(distance)+ " Metal: "+String(val));

  } else if (state == 2) {
    myservo.attach(8);
    myservo.write(180);
  }
}
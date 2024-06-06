//We wrote this code to operate the mechanism placed under the drone and to control various parts.
//The mechanism includes one Arduino Uno (transmitter), one metal sensor (FLC 100), one distance sensor (LiDAR), and one servo motor.
//This mechanism is designed to lower the metal and distance sensors towards the ground after the drone takes off.
//We used NFR24L01 to communication between two arduino unos.
//The transmitter arduino is on our drone.
//The reciever arduino is connected with our computer.


#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#define CE_PIN 9
#define CSN_PIN 10
#define button 4   
#define refbutton 5   //Referance button.

int state = 0;
int currentstate = 0;
float referance = 0;
float val = 0;
int buttonpress = 0;

RF24 radio(CE_PIN, CSN_PIN);
const byte address[][6] = { "00001", "00002" };

void setup() {
  pinMode(button, INPUT);   //Lidar (distance) sensor button.
  Serial.begin(115200);
  radio.begin();
  radio.openWritingPipe(address[0]);
  radio.openReadingPipe(0, address[1]);
  radio.setPALevel(RF24_PA_HIGH);
  radio.setDataRate(RF24_2MBPS);   //To fast communication between NRF's.
  radio.startListening();
  Serial.print("Start");
  pinMode(3, OUTPUT);   //Buzzer pin.
  pinMode(refbutton, INPUT);   //Referance button pin.

  digitalWrite(3, HIGH);
  delay(100);
  digitalWrite(3, LOW);
  delay(100);

  digitalWrite(3, HIGH);
  delay(100);
  digitalWrite(3, LOW);
  delay(100);
}

void loop() {
  delay(5);
  if (radio.available()) {   //If the communication between rf's is true. 
    float receivedData[2];
    radio.read(&receivedData, sizeof(receivedData));
    val = receivedData[0];
    int distance = (int)receivedData[1];

    Serial.println("Distance: " + String(distance) + " Metal: " + String(val));
  }

  if (digitalRead(button) == 1) {
    Serial.print("State Change");
    radio.stopListening();

    while (digitalRead(button) == 1);
    delay(50);

    switch (state) {
      //
      case 0:
        {
          state = 1;
        }
        break;
      case 1:
        {
          state = 2;
        }
        break;
      case 2:
        {
          state = 0;
        }
        break;
    }
    while (state != currentstate) {
      switch (state) {
        //After the drone completes its takeoff, press the button to lower the mechanism.
        case 0:
          {   
            buttonpress = 0; 
            char dataToSend[] = "changestate:0";
            radio.write(&dataToSend, sizeof(dataToSend));
          }
          break;
        //During the detection.
        case 1:
          {
            char dataToSend[] = "changestate:1";
            radio.write(&dataToSend, sizeof(dataToSend));
          }
          break;
        //After detection is over, to get the mechanism up.
        case 2:
          {
            buttonpress = 0;
            char dataToSend[] = "changestate:2";
            radio.write(&dataToSend, sizeof(dataToSend));
          }
          break;
      }

      radio.startListening();
      delay(50);
      char msg[32];
      radio.read(&msg, 32);
      String str(msg);
      if (str.indexOf(":") > 0) {
        currentstate = str.substring(str.indexOf(":") + 1).toInt();
      }
      delay(50);
      radio.stopListening();
    }

    delay(50);
    Serial.print("" + String(state));
    radio.startListening();
    delay(5);
  }
  if (digitalRead(refbutton) == 1) {
    referance = val;   //We made it because the metal values can change depend on the distance. Therefore, even if there is no any metal piece, the metal sensor can falsely detect metal as if it is there.
    buttonpress = 1;
    Serial.println("Referance: " + String(referance));
    delay(20);
  }
  if (buttonpress == 1) {   //If referance button pressed.
    //This is because metals have both positive and negative poles.
    if (val >= (referance + 7) || val <= (referance - 7)) { 
      digitalWrite(3, HIGH);
      delay(20);
    } else {
      digitalWrite(3, LOW);
      delay(20);
    }
  }

}

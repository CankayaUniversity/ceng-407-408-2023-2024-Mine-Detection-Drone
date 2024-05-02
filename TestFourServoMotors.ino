// This code is about testing the servo motors before build the drone.
// This code does not include controller.
// With this code, we control the servo motors via the computer.
// This code is written in Arduino IDE

#include <Servo.h>

Servo servoM1;
Servo servoM2;
Servo servoM3;
Servo servoM4;

int pos1 = 0;
int pos2 = 0;
int pos3 = 0;
int pos4 = 0;

bool isRunning = false; // To check the servo motors are working or not.
int maxPos = 120;
int minPos = 0;
int stepSize = 0;
int currentSpeed = 0;

void setup() {
  servoM1.attach(8);  //PIN 8 on arduino mega
  servoM2.attach(9);  //PIN 9 on arduino mega
  servoM3.attach(10); //PIN 10 on arduino mega
  servoM4.attach(11); //PIN 11 on arduino mega

  servoM1.write(pos1);
  servoM2.write(pos2);
  servoM3.write(pos3);
  servoM4.write(pos4);

  Serial.begin(115200); // To start the serial communication. 115200 baud rate is for arduino mega.
  Serial.flush();
}

void loop() {
  if (Serial.available()) {
    Serial.print("setup\n");
    int data = Serial.read();
    while(isRunning){
      if (pos1 < currentSpeed) {
      pos1 += 1;
      } else if (pos1 > currentSpeed) {
        pos1 -= 1;
      }

      if (pos2 < currentSpeed) {
        pos2 += 1;
      } else if (pos2 > currentSpeed) {
        pos2 -= 1;
      }

      if (pos3 < currentSpeed) {
        pos3 += 1;
      } else if (pos3 > currentSpeed) {
        pos3 -= 1;
      }

      if (pos4 < currentSpeed) {
        pos4 += 1;
      } else if (pos4 > currentSpeed) {
        pos4 -= 1;
      }
      servoM1.write(pos1);
      servoM2.write(pos2);
      servoM3.write(pos3);
      servoM4.write(pos4);

      delay(15);

      Serial.print("stop ready\n");
      if(Serial.available()){
        data=Serial.read();
        if (data == 's') {  // Stop the servo motors
          Serial.print("start ready\n");
          Serial.flush();
          currentSpeed = 0;
          while(pos1>0 || pos2>0 || pos3>0 || pos4>0)
          {
            servoM1.write(pos1);
            servoM2.write(pos2);
            servoM3.write(pos3);
            servoM4.write(pos4);            
            delay(15); 
            pos1-=1;
            pos2-=1;
            pos3-=1;
            pos4-=1;
              
          }
          isRunning = false;
        }
        if (data == 'u') {  //Speed up the servo motors
          Serial.print("speeddup\n");
          Serial.flush();
          if(currentSpeed<=120){
            currentSpeed+=20;
          }
        }
        if (data == 'd') {  //Slow down the servo motors
          Serial.print("speeddown\n");
          Serial.flush();
          if(currentSpeed>=60){
            currentSpeed-=20;
          }
        }
      }
    }
    Serial.print("start ready\n");
    if (data == 'r') {  //To ready before the start the servo motors.
      isRunning = true;
      currentSpeed=40;
      Serial.flush();
    }
  }
  else 
  {
    Serial.println("1\n");
  }
}
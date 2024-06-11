#include <Servo.h>

Servo servo1; // Left - Right control
Servo servo2; // Up - Down control

int servo1Pin = 9; //The pin that connect with servo1.
int servo2Pin = 10; //The pin that connect with servo2.

int position1 = 90; // Beginning position for servo 1  (middle position)
int position2 = 90; // Beginning position for servo 2  (middle position)

void setup() {
  Serial.begin(9600);
  servo1.attach(servo1Pin);
  servo2.attach(servo2Pin);
  servo1.write(position1);
  servo2.write(position2);
  Serial.println("Servo Control Start. W: UP, S: Down, A: Left, D: Right");
}

void loop() {

  while (Serial.available() > 0) {
    char input = Serial.read();
    if (input == 'w' || input == 'W') {
      position2 = constrain(position2 + 5, 0, 180); // Move down
      servo2.write(position2);
    } else if (input == 's' || input == 'S') {
      position2 = constrain(position2 - 5, 0, 180); // Move up
      servo2.write(position2);
    } else if (input == 'a' || input == 'A') {
      position1 = constrain(position1 - 5, 0, 180); // Move to left
      servo1.write(position1);
    } else if (input == 'd' || input == 'D') {
      position1 = constrain(position1 + 5, 0, 180); // Move to right
      servo1.write(position1);
    }
  }

  delay(100);
}
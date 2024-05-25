//This code will start in calibration mode.

#include <Wire.h>     //Using this library to communicate with the gyro.
#include <EEPROM.h>   //Using this library to store information onto the EEPROM

//Global variables
byte lastChannel, lastChannel2, lastChannel3, lastChannel4;
byte eepromData[36], start, data;
boolean newFunction, angle;
volatile int receiverInput1, receiverInput2, receiverInput3, receiverInput4;
int esc1, esc2, esc3, esc4;
int receiverInput[5];
int counterLoop, gyroAddress, counterVibration;
int temperature;
long accX, accY, accZ, accTotalVector[20], accVector, totalResult;
unsigned long timerChannel, timerChannel2, timerChannel3, timerChannel4;
unsigned long escTimerLoop;
unsigned long timerZero, timer, timer2, timer3, timer4;
unsigned long currentTime;
int accAxis[4], gyroAxis[4];
double gyroPitch, gyroRoll, gyroYaw;
float anglePitch, angleRoll;
float angleRollAcc, anglePitchAcc;
int calibration;
double gyroAxisCalibration[4];

//Setup routine
void setup(){

  Serial.begin(57600);  //Start the serial port.
  Wire.begin();  //Start the wire library as master
  TWBR = 12;  //Set the I2C clock speed to 400kHz.

  //Arduino Uno pins default to inputs.
  DDRD |= B11110000;  //Configure digital port 4, 5, 6 and 7 as output.
  DDRB |= B00010000;  //Configure digital port 12 as output.

  PCICR |= (1 << PCIE0);
  //Set PCINT0(digital input 8), PCINT1(digital input 9), PCINT2(digital input 10), PCINT3(digital input 11) to trigger an interrupt on state change.                                                               // set PCIE0 to enable PCMSK0 scan.
  PCMSK0 |= (1 << PCINT0);
  PCMSK0 |= (1 << PCINT1);
  PCMSK0 |= (1 << PCINT2);
  PCMSK0 |= (1 << PCINT3);

  for(data = 0; data <= 35; data++)eepromData[data] = EEPROM.read(data); //Read EEPROM for faster data access

  gyroAddress = eepromData[32];  //Store the gyro address in the variable.

  setGyroRegisters();  //Set the specific gyro registers.                                                               

  //Check the EEPROM signature to make sure that the setup program is executed.
  while(eepromData[33] != 'J' || eepromData[34] != 'M' || eepromData[35] != 'B'){
    delay(500);
    digitalWrite(12, !digitalRead(12));  //Change the led status to indicate error.
  }

  waitReceiver();  //Wait until the receiver is active.
  
  timerZero = micros();  //Set the timerZero for the first loop.

  while(Serial.available())data = Serial.read();  //Empty the serial buffer.
  data = 0;
}

//Main program loop
void loop(){
  
  while(timerZero + 4000 > micros());
  timerZero = micros();

  if(Serial.available() > 0){

    data = Serial.read();
    delay(100);
    
    while(Serial.available() > 0)counterLoop = Serial.read();  //Empty the Serial buffer.
    
    newFunction = true;  //Set the new request flag.
    counterLoop = 0;                                                               
    calibration = 0;
    start = 0;
    angle = false;
    
    //Confirm the choice on the serial monitor.
    if(data == 'r')Serial.println("Reading receiver signals.");
    if(data == 'a')Serial.println("Print the quadcopter angles.");
    if(data == 'a')Serial.println("Gyro calibration starts in 2 seconds (don't move the quadcopter).");
    if(data == '1')Serial.println("Test motor 1 (right front CCW.)");
    if(data == '2')Serial.println("Test motor 2 (right rear CW.)");
    if(data == '3')Serial.println("Test motor 3 (left rear CCW.)");
    if(data == '4')Serial.println("Test motor 4 (left front CW.)");
    if(data == '5')Serial.println("Test all motors together");

    //Create a small delay so the message stays visible for 2.5 seconds.
    //We don't want the ESC's to beep and have to send a 1000us pulse to the ESC's.
    for(counterVibration = 0; counterVibration < 625; counterVibration++){
      delay(3);
      esc1 = 1000;
      esc2 = 1000;
      esc3 = 1000;
      esc4 = 1000;
      escOutput();
    }
    counterVibration = 0;
  }

  receiverInput3 = convertReceiverChannel(3);  //Convert the actual receiver signals for throttle to the standard 1000 - 2000us.
  if(receiverInput3 < 1025)newFunction = false;

  //Run the ESC calibration program to start with.
  if(data == 0 && newFunction == false){  //Only start the calibration mode at first start. 
    receiverInput3 = convertReceiverChannel(3);  //Convert the actual receiver signals for throttle to the standard 1000 - 2000us.
    esc1 = receiverInput3;  //Set the pulse for motor 1 equal to the throttle channel.
    esc2 = receiverInput3;  //Set the pulse for motor 2 equal to the throttle channel.
    esc3 = receiverInput3;  //Set the pulse for motor 3 equal to the throttle channel.
    esc4 = receiverInput3;  //Set the pulse for motor 4 equal to the throttle channel.
    escOutput();
  }


  //When user sends a 'r' print the receiver signals.
  if(data == 'r'){
    counterLoop ++;
    receiverInput1 = convertReceiverChannel(1);  //Convert the actual receiver signals for pitch to the standard 1000 - 2000us.
    receiverInput2 = convertReceiverChannel(2);  //Convert the actual receiver signals for roll to the standard 1000 - 2000us.
    receiverInput3 = convertReceiverChannel(3);  //Convert the actual receiver signals for throttle to the standard 1000 - 2000us.
    receiverInput4 = convertReceiverChannel(4);  //Convert the actual receiver signals for yaw to the standard 1000 - 2000us.

    if(counterLoop == 125){
      printSignals();  //Print the receiver values on the serial monitor.
      counterLoop = 0;
    }

    if(receiverInput3 < 1050 && receiverInput4 < 1050)start = 1;  //For starting the motors: throttle low and yaw left.
    if(start == 1 && receiverInput3 < 1050 && receiverInput4 > 1450)start = 2;  //When yaw stick is back in the center position start the motors.
    if(start == 2 && receiverInput3 < 1050 && receiverInput4 > 1950)start = 0;  //Stopping the motors: throttle low and yaw right.

    esc1 = 1000;
    esc2 = 1000;
    esc3 = 1000;
    esc4 = 1000
    escOutput();
  }


  //When user sends a '1, 2, 3, 4 or 5 test the motors.
  if(data == '1' || data == '2' || data == '3' || data == '4' || data == '5'){
    counterLoop ++;

    if(newFunction == true && counterLoop == 250){  //Wait for the throttle to be set to 0.
      Serial.print("Set throttle to 1000 (low). It's now set to: ");
      Serial.println(receiverInput3);  //Print the actual throttle position.
      counterLoop = 0;
    }

    if(newFunction == false){  //When the throttle was in the lowest position do this.
      receiverInput3 = convertReceiverChannel(3);  //Convert the actual receiver signals for throttle to the standard 1000 - 2000us.

      if(data == '1' || data == '5')esc1 = receiverInput3;
      else esc1 = 1000;
      if(data == '2' || data == '5')esc2 = receiverInput3;
      else esc2 = 1000;
      if(data == '3' || data == '5')esc3 = receiverInput3;
      else esc3 = 1000;
      if(data == '4' || data == '5')esc4 = receiverInput3;
      else esc4 = 1000;

      escOutput();

      //For balancing the propellors it's possible to use the accelerometer to measure the vibrations.
      if(eepromData[31] == 1){  //The MPU-6050 is installed

        Wire.beginTransmission(gyroAddress);  //Start communication with the gyro.
        Wire.write(0x3B);  //Start reading @ register 43h and auto increment with every read.
        Wire.endTransmission();
        Wire.requestFrom(gyroAddress,6);  //Request 6 bytes from the gyro.

        while(Wire.available() < 6);  //Wait until the 6 bytes are received.

        //Add the low and high byte to the accX, accY, accZ variables.
        accX = Wire.read()<<8|Wire.read();
        accY = Wire.read()<<8|Wire.read();
        accZ = Wire.read()<<8|Wire.read();

        accTotalVector[0] = sqrt((accX*accX)+(accY*accY)+(accZ*accZ));  //Calculate the total accelerometer vector.

        accVector = accTotalVector[0];  //Copy the total vector to the accelerometer average vector variable.

        for(start = 16; start > 0; start--){  //To create an array of accelrometer vectors.
          accTotalVector[start] = accTotalVector[start - 1];  //Shift every variable one position up in the array.
          accVector += accTotalVector[start];  //Add the array value to the accVector variable.
        }

        accVector /= 17;  //To get the avarage total accelerometer vector.

        if(counterVibration < 20){
          counterVibration ++;
          totalResult += abs(accTotalVector[0] - accVector);
        }
        else{
          counterVibration = 0;
          Serial.println(totalResult/50);
          totalResult = 0;
        }
      }
    }
  }

  //When user sends a 'a' display the quadcopter angles.
  if(data == 'a'){

    if(calibration != 2000){

      Serial.print("Calibrating the gyro");

      //To determine the average gyro offset (calibration).
      for (calibration = 0; calibration < 2000 ; calibration ++){      

        if(calibration % 125 == 0){
          digitalWrite(12, !digitalRead(12));   //Change the led status to indicate calibration.
          Serial.print(".");
        }

        gyroSignal();  //Read the gyro output.

        gyroAxisCalibration[1] += gyroAxis[1];  //Ad roll value to gyro_roll_cal.
        gyroAxisCalibration[2] += gyroAxis[2];  //Ad pitch value to gyro_pitch_cal.
        gyroAxisCalibration[3] += gyroAxis[3];  //Ad yaw value to gyro_yaw_cal.
        
        //To avoid the ESCs beeping annoyingly, set them to a 1000µs pulse while calibrating the gyro.
        PORTD |= B11110000;  //Set digital poort 4, 5, 6 and 7 high.
        delayMicroseconds(1000);
        PORTD &= B00001111;  //Set digital poort 4, 5, 6 and 7 low.
        delay(3);
      }
      Serial.println(".");

      //To get the average gyro offset.
      gyroAxisCalibration[1] /= 2000;
      gyroAxisCalibration[2] /= 2000;
      gyroAxisCalibration[3] /= 2000;
    }
    else{

      //To avoid the ESCs beeping annoyingly, set them to a 1000µs pulse while calibrating the gyro.
      PORTD |= B11110000;  //Set digital poort 4, 5, 6 and 7 high.
      delayMicroseconds(1000);
      PORTD &= B00001111;  //Set digital poort 4, 5, 6 and 7 low.

      gyroSignal();  //Let's get the current gyro data.

      //Gyro angle calculations
      //0.0000611 = 1 / (250Hz / 65.5)
      anglePitch += gyroPitch * 0.0000611;  //Calculate the traveled pitch angle and add this to the anglePitch variable.
      angleRoll += gyroRoll * 0.0000611;  //Calculate the traveled roll angle and add this to the angleRoll variable.

      //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians
      anglePitch -= angleRoll * sin(gyroYaw * 0.000001066);  //If the IMU has yawed transfer the roll angle to the pitch angel.
      angleRoll += anglePitch * sin(gyroYaw * 0.000001066);  //If the IMU has yawed transfer the pitch angle to the roll angel.

      //Accelerometer angle calculations
      accTotalVector[0] = sqrt((accX*accX)+(accY*accY)+(accZ*accZ));  //Calculate the total accelerometer vector.

      //57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians
      anglePitchAcc = asin((float)accY/accTotalVector[0])* 57.296;  //Calculate the pitch angle.
      angleRollAcc = asin((float)accX/accTotalVector[0])* -57.296;  //Calculate the roll angle.
      
      if(!angle){
        anglePitch = anglePitchAcc;
        angleRoll = angleRollAcc;
        angle = true;
      }
      else{
        anglePitch = anglePitch * 0.9996 + anglePitchAcc * 0.0004;  //Correct the drift of the gyro pitch angle with the accelerometer pitch angle.
        angleRoll = angleRoll * 0.9996 + angleRollAcc * 0.0004;  //Correct the drift of the gyro roll angle with the accelerometer roll angle.
      }

      //We can't print all the data at once. This takes to long and the angular readings will be off.
      if(counterLoop == 0)Serial.print("Pitch: ");
      if(counterLoop == 1)Serial.print(anglePitch ,0);
      if(counterLoop == 2)Serial.print(" Roll: ");
      if(counterLoop == 3)Serial.print(angleRoll ,0);
      if(counterLoop == 4)Serial.print(" Yaw: ");
      if(counterLoop == 5)Serial.println(gyroYaw / 65.5 ,0);

      counterLoop ++;
      if(counterLoop == 60)counterLoop = 0;      
    }
  }
}

ISR(PCINT0_vect){

  currentTime = micros();

  //Channel 1
  if(PINB & B00000001){
    if(lastChannel == 0){  //Input 8 changed from 0 to 1.
      lastChannel = 1;
      timer = currentTime;
    }
  }
  else if(lastChannel == 1){  //Input 8 is not high and changed from 1 to 0.
    lastChannel = 0;
    receiverInput[1] = currentTime - timer;
  }

  //Channel 2
  if(PINB & B00000010 ){
    if(lastChannel2 == 0){  //Input 9 changed from 0 to 1.
      lastChannel2 = 1;
      timer2 = currentTime;
    }
  }
  else if(lastChannel2 == 1){  //Input 9 is not high and changed from 1 to 0.
    lastChannel2 = 0;
    receiverInput[2] = currentTime - timer2;
  }

  //Channel 3
  if(PINB & B00000100 ){
    if(lastChannel3 == 0){  //Input 10 changed from 0 to 1.
      lastChannel3 = 1;
      timer3 = currentTime;
    }
  }
  else if(lastChannel3 == 1){  //Input 10 is not high and changed from 1 to 0.
    lastChannel3 = 0;
    receiverInput[3] = currentTime - timer3;
  }

  //Channel 4
  if(PINB & B00001000 ){
    if(lastChannel4 == 0){  //Input 11 changed from 0 to 1.
      lastChannel4 = 1;
      timer4 = currentTime;
    }
  }
  else if(lastChannel4 == 1){  //Input 11 is not high and changed from 1 to 0.
    lastChannel4 = 0;
    receiverInput[4] = currentTime - timer4;
  }
}

//Check if the receiver values are valid within 10 seconds
void waitReceiver(){
  byte zero = 0;
  while(zero < 15){
    if(receiverInput[1] < 2100 && receiverInput[1] > 900)zero |= 0b00000001;  //Set bit 0 if the receiver pulse 1 is within the 900 - 2100 range
    if(receiverInput[2] < 2100 && receiverInput[2] > 900)zero |= 0b00000010;  //Set bit 1 if the receiver pulse 2 is within the 900 - 2100 range
    if(receiverInput[3] < 2100 && receiverInput[3] > 900)zero |= 0b00000100;  //Set bit 2 if the receiver pulse 3 is within the 900 - 2100 range
    if(receiverInput[4] < 2100 && receiverInput[4] > 900)zero |= 0b00001000;  //Set bit 3 if the receiver pulse 4 is within the 900 - 2100 range
    delay(500);
  }
}

//This part converts the actual receiver signals to a standardized 1000 – 1500 – 2000 microsecond value.
//The stored data in the EEPROM is used.
int convertReceiverChannel(byte function){

  byte channel, reverse;
  int low, center, high, actual;
  int difference;

  channel = eepromData[function + 23] & 0b00000111;  //What channel corresponds with the specific function
  
  if(eepromData[function + 23] & 0b10000000)reverse = 1;  //Reverse channel when most significant bit is set
  else reverse = 0;  //If the most significant is not set there is no reverse

  actual = receiverInput[channel];
  low = (eepromData[channel * 2 + 15] << 8) | eepromData[channel * 2 + 14];  //Store the low value for the specific receiver input channel
  center = (eepromData[channel * 2 - 1] << 8) | eepromData[channel * 2 - 2]; //Store the center value for the specific receiver input channel
  high = (eepromData[channel * 2 + 7] << 8) | eepromData[channel * 2 + 6];   //Store the high value for the specific receiver input channel

  if(actual < center){
    if(actual < low)actual = low;
    difference = ((long)(center - actual) * (long)500) / (center - low);  //Calculate and scale the actual value to a 1000 - 2000us value
    if(reverse == 1)return 1500 + difference;  //If the channel is reversed
    else return 1500 - difference;  //If the channel is not reversed
  }
  else if(actual > center){
    if(actual > high)actual = high;
    difference = ((long)(actual - center) * (long)500) / (high - center);  //Calculate and scale the actual value to a 1000 - 2000us value
    if(reverse == 1)return 1500 - difference;  //If the channel is reversed
    else return 1500 + difference;  //If the channel is not reversed
  }
  else return 1500;
}

void printSignals(){
  Serial.print("Start:");
  Serial.print(start);

  Serial.print("  Roll:");
  if(receiverInput1 - 1480 < 0)Serial.print("<<<");
  else if(receiverInput1 - 1520 > 0)Serial.print(">>>");
  else Serial.print("-+-");
  Serial.print(receiverInput1);

  Serial.print("  Pitch:");
  if(receiverInput2 - 1480 < 0)Serial.print("^^^");
  else if(receiverInput2 - 1520 > 0)Serial.print("vvv");
  else Serial.print("-+-");
  Serial.print(receiverInput2);

  Serial.print("  Throttle:");
  if(receiverInput3 - 1480 < 0)Serial.print("vvv");
  else if(receiverInput3 - 1520 > 0)Serial.print("^^^");
  else Serial.print("-+-");
  Serial.print(receiverInput3);

  Serial.print("  Yaw:");
  if(receiverInput4 - 1480 < 0)Serial.print("<<<");
  else if(receiverInput4 - 1520 > 0)Serial.print(">>>");
  else Serial.print("-+-");
  Serial.println(receiverInput4);
}

void escOutput(){
  timerZero = micros();
  PORTD |= B11110000;  //Set port 4, 5, 6 and 7 high at once
  timerChannel = esc1 + timerZero;  //Calculate the time when digital port 4 is set low.
  timerChannel2 = esc2 + timerZero;  //Calculate the time when digital port 5 is set low.
  timerChannel3 = esc3 + timerZero;  //Calculate the time when digital port 6 is set low.
  timerChannel4 = esc4 + timerZero;  //Calculate the time when digital port 7 is set low.

  while(PORTD >= 16){  //Execute the loop until digital port 4 to 7 is low.
    escTimerLoop = micros();
    if(timerChannel <= escTimerLoop)PORTD &= B11101111;  //When the delay time is expired, digital port 4 is set low.
    if(timerChannel2 <= escTimerLoop)PORTD &= B11011111;  //When the delay time is expired, digital port 5 is set low.
    if(timerChannel3 <= escTimerLoop)PORTD &= B10111111;  //When the delay time is expired, digital port 6 is set low.
    if(timerChannel4 <= escTimerLoop)PORTD &= B01111111;  //When the delay time is expired, digital port 7 is set low.
  }
}

void setGyroRegisters(){
  
  //Setup the MPU-6050
  if(eepromData[31] == 1){
    Wire.beginTransmission(gyroAddress);  //Start communication with the address found during search.
    Wire.write(0x6B);
    Wire.write(0x00);  //Set the register bits as 00000000 to activate the gyro
    Wire.endTransmission();

    Wire.beginTransmission(gyroAddress);  //Start communication with the address found during search.
    Wire.write(0x1B);
    Wire.write(0x08);  //Set the register bits as 00001000 (500dps full scale)
    Wire.endTransmission();

    Wire.beginTransmission(gyroAddress);  //Start communication with the address found during search.
    Wire.write(0x1C);
    Wire.write(0x10);  //Set the register bits as 00010000 (+/- 8g full scale range)
    Wire.endTransmission();

    //Let's perform a random register check to see if the values are written correct
    Wire.beginTransmission(gyroAddress);  //Start communication with the address found during search
    Wire.write(0x1B);
    Wire.endTransmission();
    Wire.requestFrom(gyroAddress, 1);  //Request 1 bytes from the gyro

    while(Wire.available() < 1);  //Wait until the 6 bytes are received
    if(Wire.read() != 0x08){
      digitalWrite(12,HIGH);  //Turn on the warning led
      while(1)delay(10);  //Stay in this loop for ever
    }

    Wire.beginTransmission(gyroAddress);  //Start communication with the address found during search
    Wire.write(0x1A);
    Wire.write(0x03);  //Set the register bits as 00000011 (Set Digital Low Pass Filter to ~43Hz)
    Wire.endTransmission();

  }  
}

void gyroSignal(){
  //Read the MPU-6050
  if(eepromData[31] == 1){
    Wire.beginTransmission(gyroAddress);  //Start communication with the gyro.
    Wire.write(0x3B);  //Start reading @ register 43h and auto increment with every read.
    Wire.endTransmission();
    Wire.requestFrom(gyroAddress,14);  //Request 14 bytes from the gyro.
    while(Wire.available() < 14);  //Wait until the 14 bytes are received.
    //Add the low and high byte to the accX, accY, accZ variables.
    accAxis[1] = Wire.read()<<8|Wire.read();
    accAxis[2] = Wire.read()<<8|Wire.read();
    accAxis[3] = Wire.read()<<8|Wire.read();
    temperature = Wire.read()<<8|Wire.read();  //Add the low and high byte to the temperature variable.
    //Read high and low part of the angular data.
    gyroAxis[1] = Wire.read()<<8|Wire.read();
    gyroAxis[2] = Wire.read()<<8|Wire.read();
    gyroAxis[3] = Wire.read()<<8|Wire.read();
  }

  if(calibration == 2000){
    //Only compensate after the calibration.
    gyroAxis[1] -= gyroAxisCalibration[1];
    gyroAxis[2] -= gyroAxisCalibration[2];
    gyroAxis[3] -= gyroAxisCalibration[3];
  }

  gyroRoll = gyroAxis[eepromData[28] & 0b00000011];  //Set gyroRoll to the correct axis that was stored in the EEPROM.
  if(eepromData[28] & 0b10000000)gyroRoll *= -1;  //Invert gyroRoll if the MSB of EEPROM bit 28 is set.
  gyroPitch = gyroAxis[eepromData[29] & 0b00000011];  //Set gyroPitch to the correct axis that was stored in the EEPROM.
  if(eepromData[29] & 0b10000000)gyroPitch *= -1;  //Invert gyroPitch if the MSB of EEPROM bit 29 is set.
  gyroYaw = gyroAxis[eepromData[30] & 0b00000011];  //Set gyroYaw to the correct axis that was stored in the EEPROM.
  if(eepromData[30] & 0b10000000)gyroYaw *= -1;  //Invert gyroYaw if the MSB of EEPROM bit 30 is set.

  accX = accAxis[eepromData[29] & 0b00000011];  //Set accX to the correct axis that was stored in the EEPROM.
  if(eepromData[29] & 0b10000000)accX *= -1;  //Invert accX if the MSB of EEPROM bit 29 is set.
  accY = accAxis[eepromData[28] & 0b00000011];  //Set accY to the correct axis that was stored in the EEPROM.
  if(eepromData[28] & 0b10000000)accY *= -1;  //Invert accY if the MSB of EEPROM bit 28 is set.
  accZ = accAxis[eepromData[30] & 0b00000011];  //Set accZ to the correct axis that was stored in the EEPROM.
  if(eepromData[30] & 0b10000000)accZ *= -1;  //Invert accZ if the MSB of EEPROM bit 30 is set.
}
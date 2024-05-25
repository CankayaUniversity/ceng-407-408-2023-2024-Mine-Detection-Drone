#include <Wire.h>  //To communicate with the gyro
#include <EEPROM.h>  //To store information onto the EEPROM

//Global Variables
byte lastChannel, lastChannel2, lastChannel3, lastChannel4;
byte lowByte, highByte, type, gyroAddress, error, clockSpeed;
byte assignChannel1, assignChannel2, assignChannel3, assignChannel4;
byte rollAxis, pitchAxis, yawAxis;
byte receiverCheck, gyroCheck;
volatile int receiverInputChannel1, receiverInputChannel2, receiverInputChannel3, receiverInputChannel4;
int centerChannel1, centerChannel2, centerChannel3, centerChannel4;
int highChannel1, highChannel2, highChannel3, highChannel4;
int lowChannel1, lowChannel2, lowChannel3, lowChannel4;
int address, calibration;
unsigned long timer, timer1, timer2, timer3, timer4, currentTime;
float gyroRoll, gyroPitch, gyroYaw;
float gyroRollCalibration, gyroPitchCalibration, gyroYawCalibration;


//Setup routine
void setup(){
  pinMode(12, OUTPUT);
  //Arduino (Atmega) pins default to inputs, so they don't need to be explicitly declared as inputs
  PCICR |= (1 << PCIE0);  // set PCIE0 to enable PCMSK0 scan
  //Set PCINT0 (digital input 8), PCINT1 (digital input 9), PCINT2 (digital input 10), PCINT3 (digital input 11) to trigger an interrupt on state change
  PCMSK0 |= (1 << PCINT0);
  PCMSK0 |= (1 << PCINT1);
  PCMSK0 |= (1 << PCINT2);
  PCMSK0 |= (1 << PCINT3);
  Wire.begin();
  Serial.begin(57600);
  delay(250);
}

void loop(){

  intro();
  
  Serial.println(F(""));
  Serial.println(F("==================================================="));
  Serial.println(F("System check"));
  Serial.println(F("==================================================="));
  delay(1000);
  Serial.println(F("Checking I2C clock speed."));
  delay(1000);
  
  TWBR = 12;  //Set the I2C clock speed to 400kHz.
  
  #if F_CPU == 16000000L  //If the clock speed is 16MHz include the next code line when compiling
    clockSpeed = 1;
  #endif

  if(TWBR == 12 && clockSpeed){
    Serial.println(F("I2C clock speed is correctly set to 400kHz."));
  }
  else{
    Serial.println(F("I2C clock speed is not set to 400kHz. (ERROR 8)"));
    error = 1;
  }
  
  if(error == 0){
    Serial.println(F(""));
    Serial.println(F("==================================================="));
    Serial.println(F("Transmitter setup"));
    Serial.println(F("==================================================="));
    delay(1000);
    Serial.print(F("Checking for valid receiver signals."));
    waitReceiver();
    Serial.println(F(""));
  }

  //Quit the program in case of an error
  if(error == 0){

    delay(2000);
    
    Serial.println(F("Place all sticks and subtrims in the center position within 10 seconds."));

    for(int i = 9;i > 0;i--){
      delay(1000);
      Serial.print(i);
      Serial.print(" ");
    }

    Serial.println(" ");

    //Store the central stick positions
    centerChannel1 = receiverInputChannel1;
    centerChannel2 = receiverInputChannel2;
    centerChannel3 = receiverInputChannel3;
    centerChannel4 = receiverInputChannel4;

    Serial.println(F(""));
    Serial.println(F("Center positions stored."));
    Serial.print(F("Digital input 08 = "));
    Serial.println(receiverInputChannel1);
    Serial.print(F("Digital input 09 = "));
    Serial.println(receiverInputChannel2);
    Serial.print(F("Digital input 10 = "));
    Serial.println(receiverInputChannel3);
    Serial.print(F("Digital input 11 = "));
    Serial.println(receiverInputChannel4);
    Serial.println(F(""));
    Serial.println(F(""));
  }
  if(error == 0){  
    Serial.println(F("Move the throttle stick to full throttle and back to center"));

    //Check for throttle movement
    checkReceiverInputs(1);
    Serial.print(F("Throttle is connected to digital input "));
    Serial.println((assignChannel3 & 0b00000111) + 7);

    if(assignChannel3 & 0b10000000)Serial.println(F("Channel inverted = yes"));
    else Serial.println(F("Channel inverted = no"));

    waitSticks();
    
    Serial.println(F(""));
    Serial.println(F(""));
    Serial.println(F("Move the roll stick to simulate left wing up and back to center"));

    //Check for throttle movement
    checkReceiverInputs(2);
    Serial.print(F("Roll is connected to digital input "));
    Serial.println((assignChannel1 & 0b00000111) + 7);

    if(assignChannel1 & 0b10000000)Serial.println(F("Channel inverted = yes"));
    else Serial.println(F("Channel inverted = no"));

    waitSticks();
  }
  if(error == 0){

    Serial.println(F(""));
    Serial.println(F(""));
    Serial.println(F("Move the pitch stick to simulate nose up and back to center"));

    //Check for throttle movement
    checkReceiverInputs(3);
    Serial.print(F("Pitch is connected to digital input "));
    Serial.println((assignChannel2 & 0b00000111) + 7);

    if(assignChannel2 & 0b10000000)Serial.println(F("Channel inverted = yes"));
    else Serial.println(F("Channel inverted = no"));

    waitSticks();
  }
  if(error == 0){

    Serial.println(F(""));
    Serial.println(F(""));
    Serial.println(F("Move the yaw stick to simulate nose right and back to center"));

    //Check for throttle movement
    checkReceiverInputs(4);
    Serial.print(F("Yaw is connected to digital input "));
    Serial.println((assignChannel4 & 0b00000111) + 7);

    if(assignChannel4 & 0b10000000)Serial.println(F("Channel inverted = yes"));
    else Serial.println(F("Channel inverted = no"));

    waitSticks();
  }
  if(error == 0){
    Serial.println(F(""));
    Serial.println(F(""));
    Serial.println(F("Gently move all the sticks simultaneously to their extends"));
    Serial.println(F("When ready put the sticks back in their center positions"));
    //Register the min and max values of the receiver channels
    MinMax();
    Serial.println(F(""));
    Serial.println(F(""));
    Serial.println(F("High, low and center values found during setup"));
    Serial.print(F("Digital input 08 values:"));
    Serial.print(lowChannel1);
    Serial.print(F(" - "));
    Serial.print(centerChannel1);
    Serial.print(F(" - "));
    Serial.println(highChannel1);
    Serial.print(F("Digital input 09 values:"));
    Serial.print(lowChannel2);
    Serial.print(F(" - "));
    Serial.print(centerChannel2);
    Serial.print(F(" - "));
    Serial.println(highChannel2);
    Serial.print(F("Digital input 10 values:"));
    Serial.print(lowChannel3);
    Serial.print(F(" - "));
    Serial.print(centerChannel3);
    Serial.print(F(" - "));
    Serial.println(highChannel3);
    Serial.print(F("Digital input 11 values:"));
    Serial.print(lowChannel4);
    Serial.print(F(" - "));
    Serial.print(centerChannel4);
    Serial.print(F(" - "));
    Serial.println(highChannel4);
    Serial.println(F("Move stick 'nose up' and back to center to continue"));
    checkAndContinue();
  }
    
  if(error == 0){

    //What gyro is connected
    Serial.println(F(""));
    Serial.println(F("==================================================="));
    Serial.println(F("Gyro search"));
    Serial.println(F("==================================================="));
    delay(2000);
    
    Serial.println(F("Searching for MPU-6050 on address 0x68/104"));
    delay(1000);

    if(searchGyro(0x68, 0x75) == 0x68){
      Serial.println(F("MPU-6050 found on address 0x68"));
      type = 1;
      gyroAddress = 0x68;
    }
    
    if(type == 0){
      Serial.println(F("Searching for MPU-6050 on address 0x69/105"));
      delay(1000);

      if(searchGyro(0x69, 0x75) == 0x68){
        Serial.println(F("MPU-6050 found on address 0x69"));
        type = 1;
        gyroAddress = 0x69;
      }
    }
    
    if(type == 0){
      Serial.println(F("Searching for L3G4200D on address 0x68/104"));
      delay(1000);

      if(searchGyro(0x68, 0x0F) == 0xD3){
        Serial.println(F("L3G4200D found on address 0x68"));
        type = 2;
        gyroAddress = 0x68;
      }
    }
    
    if(type == 0){
      Serial.println(F("Searching for L3G4200D on address 0x69/105"));
      delay(1000);

      if(searchGyro(0x69, 0x0F) == 0xD3){
        Serial.println(F("L3G4200D found on address 0x69"));
        type = 2;
        gyroAddress = 0x69;
      }
    }
    
    if(type == 0){
      Serial.println(F("Searching for L3GD20H on address 0x6A/106"));
      delay(1000);

      if(searchGyro(0x6A, 0x0F) == 0xD7){
        Serial.println(F("L3GD20H found on address 0x6A"));
        type = 3;
        gyroAddress = 0x6A;
      }
    }
    
    if(type == 0){
     Serial.println(F("Searching for L3GD20H on address 0x6B/107"));
      delay(1000);

      if(searchGyro(0x6B, 0x0F) == 0xD7){
        Serial.println(F("L3GD20H found on address 0x6B"));
        type = 3;
        gyroAddress = 0x6B;
      }
    }
    
    if(type == 0){
      Serial.println(F("No gyro device found!!! (ERROR 3)"));
      error = 1;
    }  
    else{
      delay(3000);
      Serial.println(F(""));
      Serial.println(F("==================================================="));
      Serial.println(F("Gyro register settings"));
      Serial.println(F("==================================================="));
      startGyro(); //Setup the gyro for further use
    }
  }
  
  //If the gyro is found we can setup the correct gyro axes.
  if(error == 0){
    delay(3000);
    Serial.println(F(""));
    Serial.println(F("==================================================="));
    Serial.println(F("Gyro calibration"));
    Serial.println(F("==================================================="));
    Serial.println(F("Don't move the quadcopter!! Calibration starts in 3 seconds"));
    delay(3000);
    Serial.println(F("Calibrating the gyro, this will take +/- 8 seconds"));
    Serial.print(F("Please wait"));

    //Determine the average gyro offset (calibration).
    for (calibration = 0; calibration < 2000 ; calibration ++){
      if(calibration % 100 == 0)Serial.print(F("."));  //Print dot to indicate calibration.
      gyroSignal();
      gyroRollCalibration += gyroRoll;
      gyroPitchCalibration += gyroPitch;
      gyroYawCalibration += gyroYaw;
      delay(4);
    }

    //To get the average gyro offset.
    gyroRollCalibration /= 2000;
    gyroPitchCalibration /= 2000;
    gyroYawCalibration /= 2000;
    
    //Show the calibration results
    Serial.println(F(""));
    Serial.print(F("Axis 1 offset="));
    Serial.println(gyroRollCalibration);
    Serial.print(F("Axis 2 offset="));
    Serial.println(gyroPitchCalibration);
    Serial.print(F("Axis 3 offset="));
    Serial.println(gyroYawCalibration);
    Serial.println(F(""));
    Serial.println(F("==================================================="));
    Serial.println(F("Gyro axes configuration"));
    Serial.println(F("==================================================="));
    
    //Detect the left wing up movement
    Serial.println(F("Lift the left side of the quadcopter to a 45 degree angle within 10 seconds"));

    //Check axis movement
    checkGyroAxes(1);
    if(error == 0){
      Serial.println(F("OK!"));
      Serial.print(F("Angle detection = "));
      Serial.println(rollAxis & 0b00000011);

      if(rollAxis & 0b10000000)Serial.println(F("Axis inverted = yes"));
      else Serial.println(F("Axis inverted = no"));

      Serial.println(F("Put the quadcopter back in its original position"));
      Serial.println(F("Move stick 'nose up' and back to center to continue"));
      checkAndContinue();

      //Detect the nose up movement
      Serial.println(F(""));
      Serial.println(F(""));
      Serial.println(F("Lift the nose of the quadcopter to a 45 degree angle within 10 seconds"));

      //Check axis movement
      checkGyroAxes(2);
    }
    if(error == 0){
      Serial.println(F("OK!"));
      Serial.print(F("Angle detection = "));
      Serial.println(pitchAxis & 0b00000011);

      if(pitchAxis & 0b10000000)Serial.println(F("Axis inverted = yes"));
      else Serial.println(F("Axis inverted = no"));

      Serial.println(F("Put the quadcopter back in its original position"));
      Serial.println(F("Move stick 'nose up' and back to center to continue"));
      checkAndContinue();
      
      //Detect the nose right movement
      Serial.println(F(""));
      Serial.println(F(""));
      Serial.println(F("Rotate the nose of the quadcopter 45 degree to the right within 10 seconds"));

      //Check axis movement
      checkGyroAxes(3);
    }
    if(error == 0){
      Serial.println(F("OK!"));
      Serial.print(F("Angle detection = "));
      Serial.println(yawAxis & 0b00000011);

      if(yawAxis & 0b10000000)Serial.println(F("Axis inverted = yes"));
      else Serial.println(F("Axis inverted = no"));

      Serial.println(F("Put the quadcopter back in its original position"));
      Serial.println(F("Move stick 'nose up' and back to center to continue"));
      checkAndContinue();
    }
  }
  if(error == 0){
    Serial.println(F(""));
    Serial.println(F("==================================================="));
    Serial.println(F("LED test"));
    Serial.println(F("==================================================="));
    digitalWrite(12, HIGH);
    Serial.println(F("The LED should now be lit"));
    Serial.println(F("Move stick 'nose up' and back to center to continue"));
    checkAndContinue();
    digitalWrite(12, LOW);
  }
  
  Serial.println(F(""));
  
  if(error == 0){
    Serial.println(F("==================================================="));
    Serial.println(F("Final setup check"));
    Serial.println(F("==================================================="));
    delay(1000);

    if(receiverCheck == 0b00001111){
      Serial.println(F("Receiver channels ok"));
    }
    else{
      Serial.println(F("Receiver channel verification failed!!! (ERROR 6)"));
      error = 1;
    }

    delay(1000);

    if(gyroCheck == 0b00000111){
      Serial.println(F("Gyro axes ok"));
    }
    else{
      Serial.println(F("Gyro exes verification failed!!! (ERROR 7)"));
      error = 1;
    }
  }     
  
  if(error == 0){

    //If all is good, store the information in the EEPROM
    Serial.println(F(""));
    Serial.println(F("==================================================="));
    Serial.println(F("Storing EEPROM information"));
    Serial.println(F("==================================================="));
    Serial.println(F("Writing EEPROM"));
    delay(1000);
    Serial.println(F("Done!"));

    EEPROM.write(0, centerChannel1 & 0b11111111);
    EEPROM.write(1, centerChannel1 >> 8);
    EEPROM.write(2, centerChannel2 & 0b11111111);
    EEPROM.write(3, centerChannel2 >> 8);
    EEPROM.write(4, centerChannel3 & 0b11111111);
    EEPROM.write(5, centerChannel3 >> 8);
    EEPROM.write(6, centerChannel4 & 0b11111111);
    EEPROM.write(7, centerChannel4 >> 8);
    EEPROM.write(8, highChannel1 & 0b11111111);
    EEPROM.write(9, highChannel1 >> 8);
    EEPROM.write(10, highChannel2 & 0b11111111);
    EEPROM.write(11, highChannel2 >> 8);
    EEPROM.write(12, highChannel3 & 0b11111111);
    EEPROM.write(13, highChannel3 >> 8);
    EEPROM.write(14, highChannel4 & 0b11111111);
    EEPROM.write(15, highChannel4 >> 8);
    EEPROM.write(16, lowChannel1 & 0b11111111);
    EEPROM.write(17, lowChannel1 >> 8);
    EEPROM.write(18, lowChannel2 & 0b11111111);
    EEPROM.write(19, lowChannel2 >> 8);
    EEPROM.write(20, lowChannel3 & 0b11111111);
    EEPROM.write(21, lowChannel3 >> 8);
    EEPROM.write(22, lowChannel4 & 0b11111111);
    EEPROM.write(23, lowChannel4 >> 8);
    EEPROM.write(24, assignChannel1);
    EEPROM.write(25, assignChannel2);
    EEPROM.write(26, assignChannel3);
    EEPROM.write(27, assignChannel4);
    EEPROM.write(28, rollAxis);
    EEPROM.write(29, pitchAxis);
    EEPROM.write(30, yawAxis);
    EEPROM.write(31, type);
    EEPROM.write(32, gyroAddress);

    //Write the EEPROM signature
    EEPROM.write(33, 'J'); 
    EEPROM.write(34, 'M');
    EEPROM.write(35, 'B');
        
    
    //To make sure evrything is ok, verify the EEPROM data.
    Serial.println(F("Verify EEPROM data"));
    delay(1000);

    if(centerChannel1 != ((EEPROM.read(1) << 8) | EEPROM.read(0)))error = 1;
    if(centerChannel2 != ((EEPROM.read(3) << 8) | EEPROM.read(2)))error = 1;
    if(centerChannel3 != ((EEPROM.read(5) << 8) | EEPROM.read(4)))error = 1;
    if(centerChannel4 != ((EEPROM.read(7) << 8) | EEPROM.read(6)))error = 1;
    
    if(highChannel1 != ((EEPROM.read(9) << 8) | EEPROM.read(8)))error = 1;
    if(highChannel2 != ((EEPROM.read(11) << 8) | EEPROM.read(10)))error = 1;
    if(highChannel3 != ((EEPROM.read(13) << 8) | EEPROM.read(12)))error = 1;
    if(highChannel4 != ((EEPROM.read(15) << 8) | EEPROM.read(14)))error = 1;
    
    if(lowChannel1 != ((EEPROM.read(17) << 8) | EEPROM.read(16)))error = 1;
    if(lowChannel2 != ((EEPROM.read(19) << 8) | EEPROM.read(18)))error = 1;
    if(lowChannel3 != ((EEPROM.read(21) << 8) | EEPROM.read(20)))error = 1;
    if(lowChannel4 != ((EEPROM.read(23) << 8) | EEPROM.read(22)))error = 1;
    
    if(assignChannel1 != EEPROM.read(24))error = 1;
    if(assignChannel2 != EEPROM.read(25))error = 1;
    if(assignChannel3 != EEPROM.read(26))error = 1;
    if(assignChannel4 != EEPROM.read(27))error = 1;
    
    if(rollAxis != EEPROM.read(28))error = 1;
    if(pitchAxis != EEPROM.read(29))error = 1;
    if(yawAxis != EEPROM.read(30))error = 1;
    if(type != EEPROM.read(31))error = 1;
    if(gyroAddress != EEPROM.read(32))error = 1;
    
    if('J' != EEPROM.read(33))error = 1;
    if('M' != EEPROM.read(34))error = 1;
    if('B' != EEPROM.read(35))error = 1;
  
    if(error == 1)Serial.println(F("EEPROM verification failed!!! (ERROR 5)"));
    else Serial.println(F("Verification done"));
  }
  
  
  if(error == 0){
    Serial.println(F("Setup is finished."));
    Serial.println(F("You can now calibrate the esc's and upload the YMFC-AL code."));
  }
  else{
   Serial.println(F("The setup is aborted due to an error."));
   Serial.println(F("Check the Q and A page of the YMFC-AL project on:"));
   Serial.println(F("www.brokking.net for more information about this error."));
  }
  while(1);
}

//Search for the gyro and check the Who_am_I register
byte searchGyro(int gyroAddress, int who_am_i){

  Wire.beginTransmission(gyroAddress);
  Wire.write(who_am_i);
  Wire.endTransmission();
  Wire.requestFrom(gyroAddress, 1);

  timer = millis() + 100;

  while(Wire.available() < 1 && timer > millis());

  lowByte = Wire.read();
  address = gyroAddress;

  return lowByte;
}

void startGyro(){

  //Setup the L3G4200D or L3GD20H
  if(type == 2 || type == 3){
    Wire.beginTransmission(address);  //Start communication with the gyro with the address found during search
    Wire.write(0x20);
    Wire.write(0x0F);  //Set the register bits as 00001111 (Turn on the gyro and enable all axis)
    Wire.endTransmission();

    Wire.beginTransmission(address);  //Start communication with the gyro (adress 1101001)
    Wire.write(0x20);
    Wire.endTransmission();
    Wire.requestFrom(address, 1);  //Request 6 bytes from the gyro

    while(Wire.available() < 1);
    
    Serial.print(F("Register 0x20 is set to:"));
    Serial.println(Wire.read(),BIN);

    Wire.beginTransmission(address);  //Start communication with the gyro  with the address found during search
    Wire.write(0x23);
    Wire.write(0x90);  //Set the register bits as 10010000 (Block Data Update active & 500dps full scale)
    Wire.endTransmission();
    
    Wire.beginTransmission(address);  //Start communication with the gyro (adress 1101001)
    Wire.write(0x23);
    Wire.endTransmission();
    Wire.requestFrom(address, 1);

    while(Wire.available() < 1);
    
    Serial.print(F("Register 0x23 is set to:"));
    Serial.println(Wire.read(),BIN);

  }
  //Setup the MPU-6050
  if(type == 1){
    
    Wire.beginTransmission(address);  //Start communication with the gyro
    Wire.write(0x6B);  //PWR_MGMT_1 register
    Wire.write(0x00);  //Set to zero to turn on the gyro
    Wire.endTransmission();
    
    Wire.beginTransmission(address);  //Start communication with the gyro
    Wire.write(0x6B);
    Wire.endTransmission();
    Wire.requestFrom(address, 1);

    while(Wire.available() < 1);

    Serial.print(F("Register 0x6B is set to:"));
    Serial.println(Wire.read(),BIN);
    
    Wire.beginTransmission(address);  //Start communication with the gyro
    Wire.write(0x1B);  //GYRO_CONFIG register
    Wire.write(0x08);  //Set the register bits as 00001000 (500dps full scale)
    Wire.endTransmission();
    
    Wire.beginTransmission(address);  //Start communication with the gyro (adress 1101001)
    Wire.write(0x1B);
    Wire.endTransmission();
    Wire.requestFrom(address, 1);

    while(Wire.available() < 1);

    Serial.print(F("Register 0x1B is set to:"));
    Serial.println(Wire.read(),BIN);

  }
}

void gyroSignal(){
  
  if(type == 2 || type == 3){
    Wire.beginTransmission(address);  //Start communication with the gyro
    Wire.write(168);
    Wire.endTransmission();
    Wire.requestFrom(address, 6);
   
    while(Wire.available() < 6);

    lowByte = Wire.read();  //First received byte is the low part of the angular data
    highByte = Wire.read();  //Second received byte is the high part of the angular data
    gyroRoll = ((highByte<<8)|lowByte);
    
    if(calibration == 2000)gyroRoll -= gyroRollCalibration;
    
    lowByte = Wire.read();
    highByte = Wire.read();
    gyroPitch = ((highByte<<8)|lowByte);
    
    if(calibration == 2000)gyroPitch -= gyroPitchCalibration;
    
    lowByte = Wire.read();
    highByte = Wire.read();
    gyroYaw = ((highByte<<8)|lowByte);
    
    if(calibration == 2000)gyroYaw -= gyroYawCalibration;
  }
  if(type == 1){
    Wire.beginTransmission(address);  //Start communication with the gyro
    Wire.write(0x43);
    Wire.endTransmission();
    Wire.requestFrom(address,6);

    while(Wire.available() < 6);
    
    gyroRoll=Wire.read()<<8|Wire.read();
    
    if(calibration == 2000)gyroRoll -= gyroRollCalibration;
    gyroPitch=Wire.read()<<8|Wire.read();
    if(calibration == 2000)gyroPitch -= gyroPitchCalibration;
    gyroYaw=Wire.read()<<8|Wire.read();
    if(calibration == 2000)gyroYaw -= gyroYawCalibration;
  
  }
}

//Check if a receiver input value is changing within 30 seconds
void checkReceiverInputs(byte movement){
  byte trigger = 0;
  int pulse_length;

  timer = millis() + 30000;

  while(timer > millis() && trigger == 0){

    delay(250);

    if(receiverInputChannel1 > 1750 || receiverInputChannel1 < 1250){
      trigger = 1;
      receiverCheck |= 0b00000001;
      pulse_length = receiverInputChannel1;
    }
    if(receiverInputChannel2 > 1750 || receiverInputChannel2 < 1250){
      trigger = 2;
      receiverCheck |= 0b00000010;
      pulse_length = receiverInputChannel2;
    }
    if(receiverInputChannel3 > 1750 || receiverInputChannel3 < 1250){
      trigger = 3;
      receiverCheck |= 0b00000100;
      pulse_length = receiverInputChannel3;
    }
    if(receiverInputChannel4 > 1750 || receiverInputChannel4 < 1250){
      trigger = 4;
      receiverCheck |= 0b00001000;
      pulse_length = receiverInputChannel4;
    } 
  }
  if(trigger == 0){
    error = 1;
    Serial.println(F("No stick movement detected in the last 30 seconds!!! (ERROR 2)"));
  }
  //Assign the stick to the function.
  else{
    if(movement == 1){
      assignChannel3 = trigger;
      if(pulse_length < 1250)assignChannel3 += 0b10000000;
    }
    if(movement == 2){
      assignChannel1 = trigger;
      if(pulse_length < 1250)assignChannel1 += 0b10000000;
    }
    if(movement == 3){
      assignChannel2 = trigger;
      if(pulse_length < 1250)assignChannel2 += 0b10000000;
    }
    if(movement == 4){
      assignChannel4 = trigger;
      if(pulse_length < 1250)assignChannel4 += 0b10000000;
    }
  }
}

void checkAndContinue(){
  byte continue_byte = 0;

  while(continue_byte == 0){
    if(assignChannel2 == 0b00000001 && receiverInputChannel1 > centerChannel1 + 150)continue_byte = 1;
    if(assignChannel2 == 0b10000001 && receiverInputChannel1 < centerChannel1 - 150)continue_byte = 1;
    if(assignChannel2 == 0b00000010 && receiverInputChannel2 > centerChannel2 + 150)continue_byte = 1;
    if(assignChannel2 == 0b10000010 && receiverInputChannel2 < centerChannel2 - 150)continue_byte = 1;
    if(assignChannel2 == 0b00000011 && receiverInputChannel3 > centerChannel3 + 150)continue_byte = 1;
    if(assignChannel2 == 0b10000011 && receiverInputChannel3 < centerChannel3 - 150)continue_byte = 1;
    if(assignChannel2 == 0b00000100 && receiverInputChannel4 > centerChannel4 + 150)continue_byte = 1;
    if(assignChannel2 == 0b10000100 && receiverInputChannel4 < centerChannel4 - 150)continue_byte = 1;

    delay(100);
  }
  waitSticks();
}

//Check if the transmitter sticks are in the neutral position
void waitSticks(){
  byte zero = 0;

  while(zero < 15){
    if(receiverInputChannel1 < centerChannel1 + 20 && receiverInputChannel1 > centerChannel1 - 20)zero |= 0b00000001;
    if(receiverInputChannel2 < centerChannel2 + 20 && receiverInputChannel2 > centerChannel2 - 20)zero |= 0b00000010;
    if(receiverInputChannel3 < centerChannel3 + 20 && receiverInputChannel3 > centerChannel3 - 20)zero |= 0b00000100;
    if(receiverInputChannel4 < centerChannel4 + 20 && receiverInputChannel4 > centerChannel4 - 20)zero |= 0b00001000;
    
    delay(100);
  }
}

//Checck if the receiver values are valid within 10 seconds
void waitReceiver(){
  byte zero = 0;
  timer = millis() + 10000;
  
  while(timer > millis() && zero < 15){
    if(receiverInputChannel1 < 2100 && receiverInputChannel1 > 900)zero |= 0b00000001;
    if(receiverInputChannel2 < 2100 && receiverInputChannel2 > 900)zero |= 0b00000010;
    if(receiverInputChannel3 < 2100 && receiverInputChannel3 > 900)zero |= 0b00000100;
    if(receiverInputChannel4 < 2100 && receiverInputChannel4 > 900)zero |= 0b00001000;
  
    delay(500);
    Serial.print(F("."));
  }
  if(zero == 0){
    error = 1;
  
    Serial.println(F("."));
    Serial.println(F("No valid receiver signals found!!! (ERROR 1)"));
  }
  else Serial.println(F(" OK"));
}

//Register the min and max receiver values and exit when the sticks are back in the neutral position
void MinMax(){
  byte zero = 0;
  lowChannel1 = receiverInputChannel1;
  lowChannel2 = receiverInputChannel2;
  lowChannel3 = receiverInputChannel3;
  lowChannel4 = receiverInputChannel4;
  
  while(receiverInputChannel1 < centerChannel1 + 20 && receiverInputChannel1 > centerChannel1 - 20)delay(250);
  
  Serial.println(F("Measuring endpoints...."));
  
  while(zero < 15){
    if(receiverInputChannel1 < centerChannel1 + 20 && receiverInputChannel1 > centerChannel1 - 20)zero |= 0b00000001;
    if(receiverInputChannel2 < centerChannel2 + 20 && receiverInputChannel2 > centerChannel2 - 20)zero |= 0b00000010;
    if(receiverInputChannel3 < centerChannel3 + 20 && receiverInputChannel3 > centerChannel3 - 20)zero |= 0b00000100;
    if(receiverInputChannel4 < centerChannel4 + 20 && receiverInputChannel4 > centerChannel4 - 20)zero |= 0b00001000;
    if(receiverInputChannel1 < lowChannel1)lowChannel1 = receiverInputChannel1;
    if(receiverInputChannel2 < lowChannel2)lowChannel2 = receiverInputChannel2;
    if(receiverInputChannel3 < lowChannel3)lowChannel3 = receiverInputChannel3;
    if(receiverInputChannel4 < lowChannel4)lowChannel4 = receiverInputChannel4;
    if(receiverInputChannel1 > highChannel1)highChannel1 = receiverInputChannel1;
    if(receiverInputChannel2 > highChannel2)highChannel2 = receiverInputChannel2;
    if(receiverInputChannel3 > highChannel3)highChannel3 = receiverInputChannel3;
    if(receiverInputChannel4 > highChannel4)highChannel4 = receiverInputChannel4;
    
    delay(100);
  }
}

//Check if the angular position of a gyro axis is changing within 10 seconds
void checkGyroAxes(byte movement){
  byte triggerAxis = 0;
  float gyroAngleRoll, gyroAnglePitch, gyroAngleYaw;
  //Reset all axes
  gyroAngleRoll = 0;
  gyroAnglePitch = 0;
  gyroAngleYaw = 0;
  gyroSignal();
  timer = millis() + 10000;    
  
  while(timer > millis() && gyroAngleRoll > -30 && gyroAngleRoll < 30 && gyroAnglePitch > -30 && gyroAnglePitch < 30 && gyroAngleYaw > -30 && gyroAngleYaw < 30){
    
    gyroSignal();
    
    if(type == 2 || type == 3){
      gyroAngleRoll += gyroRoll * 0.00007;
      gyroAnglePitch += gyroPitch * 0.00007;
      gyroAngleYaw += gyroYaw * 0.00007;
    }
    if(type == 1){
      gyroAngleRoll += gyroRoll * 0.0000611;
      gyroAnglePitch += gyroPitch * 0.0000611;
      gyroAngleYaw += gyroYaw * 0.0000611;
    }
    
    delayMicroseconds(3700);
  }

  //Assign the moved axis to the orresponding function (pitch, roll, yaw)
  if((gyroAngleRoll < -30 || gyroAngleRoll > 30) && gyroAnglePitch > -30 && gyroAnglePitch < 30 && gyroAngleYaw > -30 && gyroAngleYaw < 30){
    gyroCheck |= 0b00000001;
    if(gyroAngleRoll < 0)triggerAxis = 0b10000001;
    else triggerAxis = 0b00000001;
  }
  if((gyroAnglePitch < -30 || gyroAnglePitch > 30) && gyroAngleRoll > -30 && gyroAngleRoll < 30 && gyroAngleYaw > -30 && gyroAngleYaw < 30){
    gyroCheck |= 0b00000010;
    if(gyroAnglePitch < 0)triggerAxis = 0b10000010;
    else triggerAxis = 0b00000010;
  }
  if((gyroAngleYaw < -30 || gyroAngleYaw > 30) && gyroAngleRoll > -30 && gyroAngleRoll < 30 && gyroAnglePitch > -30 && gyroAnglePitch < 30){
    gyroCheck |= 0b00000100;
    if(gyroAngleYaw < 0)triggerAxis = 0b10000011;
    else triggerAxis = 0b00000011;
  }
  
  if(triggerAxis == 0){
    error = 1;
    Serial.println(F("No angular motion is detected in the last 10 seconds!!! (ERROR 4)"));
  }
  else
  if(movement == 1)rollAxis = triggerAxis;
  if(movement == 2)pitchAxis = triggerAxis;
  if(movement == 3)yawAxis = triggerAxis;
  
}

//This routine is called every time input 8, 9, 10 or 11 changed state
ISR(PCINT0_vect){
  currentTime = micros();
  //Channel 1
  if(PINB & B00000001){
    if(lastChannel == 0){  //Input 8 changed from 0 to 1
      lastChannel = 1;
      timer1 = currentTime;
    }
  }
  else if(lastChannel == 1){  //Input 8 is not high and changed from 1 to 0
    lastChannel = 0;
    receiverInputChannel1 = currentTime - timer1;
  }
  //Channel 2
  if(PINB & B00000010 ){
    if(lastChannel2 == 0){  //Input 9 changed from 0 to 1
      lastChannel2 = 1;
      timer2 = currentTime;
    }
  }
  else if(lastChannel2 == 1){  //Input 9 is not high and changed from 1 to 0
    lastChannel2 = 0;
    receiverInputChannel2 = currentTime - timer2;
  }
  //Channel 3
  if(PINB & B00000100 ){
    if(lastChannel3 == 0){  //Input 10 changed from 0 to 1
      lastChannel3 = 1;
      timer3 = currentTime;
    }
  }
  else if(lastChannel3 == 1){  //Input 10 is not high and changed from 1 to 0
    lastChannel3 = 0;
    receiverInputChannel3 = currentTime - timer3;

  }
  //Channel 4
  if(PINB & B00001000 ){
    if(lastChannel4 == 0){  //Input 11 changed from 0 to 1
      lastChannel4 = 1;
      timer4 = currentTime;
    }
  }
  else if(lastChannel4 == 1){  //Input 11 is not high and changed from 1 to 0
    lastChannel4 = 0;
    receiverInputChannel4 = currentTime - timer4;
  }
}

//Intro subroutine
void intro(){
  Serial.println(F("==================================================="));
  delay(1500);
  Serial.println(F(""));
  Serial.println(F("Your"));
  delay(500);
  Serial.println(F("  Multicopter"));
  delay(500);
  Serial.println(F("    Flight"));
  delay(500);
  Serial.println(F("      Controller"));
  delay(1000);
  Serial.println(F(""));
  Serial.println(F("YMFC-AL Setup Program"));
  Serial.println(F(""));
  Serial.println(F("==================================================="));
  delay(1500);
  Serial.println(F("For support and questions: www.brokking.net"));
  Serial.println(F(""));
  Serial.println(F("Have fun!"));
}

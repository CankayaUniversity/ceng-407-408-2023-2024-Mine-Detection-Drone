#include <Wire.h>  //To communicate with the gyro
#include <EEPROM.h>  //To store information onto the EEPROM

//PID gain and limit settings
float pidGainRollForPController = 1.3;
float pidGainRollForIController = 0.04;
float pidGainRollForDController = 18.0;

float pidGainPitchForPController = pidGainRollForPController;
float pidGainPitchForIController = pidGainRollForIController;
float pidGainPitchForDController = pidGainRollForDController;

float pidGainYawForPController = 4.0;
float pidGainYawForIController = 0.02;
float pidGainYawForDController = 0.0;

int pidRollMax = 400;
int pidPitchMax = pidRollMax;
int pidYawMax = 400;

boolean autoLevel = true;

//Global variables
byte lastChannel, lastChannel2, lastChannel3, lastChannel4;
byte eepromData[36];
byte highByte, lowByte;
volatile int receiverInput1, receiverInput2, receiverInput3, receiverInput4;
int esc1, esc2, esc3, esc4;
int throttle, batteryVoltage;
int calibration, start, gyroAddress;
int receiverInput[5];
int temperature;
int accAxis[4], gyroAxis[4];
float rollLevel, pitchLevel;

long accX, accY, accZ;
long accTotalVector;
unsigned long timerChannel1, timerChannel2, timerChannel3, timerChannel4, escTimerLoop;
unsigned long timer1, timer2, timer3, timer4, currentTime;
unsigned long loopTimer;
double gyroPitch, gyroRoll, gyroYaw;  
double gyroAxisCalibration[4];

float pidError;
float pidMemRoll, pidMemPitch, pidMemYaw;
float pidSetpointRoll, pidSetpointPitch, pidSetpointYaw;
float gyroInputRoll, gyroInputPitch, gyroInputYaw;
float pidOutputRoll, pidOutputPitch, pidOutputYaw;
float pidErrorRoll, pidErrorPitch, pidErrorYaw;
float angleRollAcc, anglePitchAcc;
float anglePitch, angleRoll;

boolean gyroAngle;

void setup(){
  Serial.begin(57600);

  for(start = 0; start <= 35; start++)eepromData[start] = EEPROM.read(start);
  start = 0;
  gyroAddress = eepromData[32];

  Wire.begin();  //Start the I2C as master.

  TWBR = 12;

  DDRD |= B11110000;  //Configure digital poort 4, 5, 6 and 7 as output
  DDRB |= B00110000;  //Configure digital poort 12 and 13 as output

  //Use the led on the Arduino for startup indication
  digitalWrite(12,HIGH);  //Turn on the warning led

  //Check the EEPROM signature to make sure that the setup program is executed
  while(eepromData[33] != 'J' || eepromData[34] != 'M' || eepromData[35] != 'B')delay(10);

  //The flight controller needs the MPU-6050 with gyro and accelerometer. If setup is completed without MPU-6050 stop the flight controller program  
  if(eepromData[31] == 2 || eepromData[31] == 3)delay(10);

  setGyroRegisters();

  for (calibration = 0; calibration < 1250 ; calibration ++){  //Wait 5 seconds before continuing.
    PORTD |= B11110000;  //Set digital poort 4, 5, 6 and 7 high.
    delayMicroseconds(1000);
    PORTD &= B00001111;  //Set digital poort 4, 5, 6 and 7 low.
    delayMicroseconds(3000);
  }

  //Let's take multiple gyro data samples so we can determine the average gyro offset (calibration).
  for (calibration = 0; calibration < 2000 ; calibration ++){
    if(calibration % 15 == 0)digitalWrite(12, !digitalRead(12));  //Change the led status to indicate calibration.
    gyroSignal();  //Read the gyro output.
    gyroAxisCalibration[1] += gyroAxis[1];  //Ad roll value to gyro roll calibration.
    gyroAxisCalibration[2] += gyroAxis[2];  //Ad pitch value to gyro pitch calibration.
    gyroAxisCalibration[3] += gyroAxis[3];  //Ad yaw value to gyro yaw calibration.

    //We don't want the esc's to be beeping annoyingly. So let's give them a 1000us puls while calibrating the gyro.
    PORTD |= B11110000;  //Set digital poort 4, 5, 6 and 7 high.
    delayMicroseconds(1000);
    PORTD &= B00001111;  //Set digital poort 4, 5, 6 and 7 low.
    delay(3);
  }
  //To get the average gyro offset.
  gyroAxisCalibration[1] /= 2000;
  gyroAxisCalibration[2] /= 2000;
  gyroAxisCalibration[3] /= 2000;

  PCICR |= (1 << PCIE0);  //Set PCIE0 to enable PCMSK0 scan.
  //Set PCINT0 (digital input 8), PCINT1 (digital input 9), PCINT2 (digital input 10), PCINT3 (digital input 11) to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT0);
  PCMSK0 |= (1 << PCINT1);
  PCMSK0 |= (1 << PCINT2);
  PCMSK0 |= (1 << PCINT3);

  //Wait until the receiver is active and the throtle is set to the lower position
  while(receiverInput3 < 990 || receiverInput3 > 1020 || receiverInput4 < 1400){
    receiverInput3 = convertReceiverChannel(3);
    receiverInput4 = convertReceiverChannel(4);
    start ++;
    //We don't want the esc's to be beeping annoyingly. So let's give them a 1000us puls while waiting for the receiver inputs
    PORTD |= B11110000;  //Set digital poort 4, 5, 6 and 7 high
    delayMicroseconds(1000);
    PORTD &= B00001111;  //Set digital poort 4, 5, 6 and 7 low
    delay(3);
    if(start == 125){
      digitalWrite(12, !digitalRead(12));  //Change the led status
      start = 0;
    }
  }
  start = 0;

  //The variable batteryVoltage holds 1050 if the battery voltage is 10.5V
  batteryVoltage = (analogRead(0) + 65) * 1.2317;

  loopTimer = micros();

  digitalWrite(12,LOW);  //Turn off the warning led
}

void loop(){

  //Gyro pid input is deg/sec.
  gyroInputRoll = (gyroInputRoll * 0.7) + ((gyroRoll / 65.5) * 0.3);
  gyroInputPitch = (gyroInputPitch * 0.7) + ((gyroPitch / 65.5) * 0.3);
  gyroInputYaw = (gyroInputYaw * 0.7) + ((gyroYaw / 65.5) * 0.3);
  
  //Gyro angle calculations
  anglePitch += gyroPitch * 0.0000611;
  angleRoll += gyroRoll * 0.0000611;

  anglePitch -= angleRoll * sin(gyroYaw * 0.000001066);  //If the IMU has yawed transfer the roll angle to the pitch angel
  angleRoll += anglePitch * sin(gyroYaw * 0.000001066);  //If the IMU has yawed transfer the pitch angle to the roll angel

  //Accelerometer angle calculations
  accTotalVector = sqrt((accX*accX)+(accY*accY)+(accZ*accZ));
  
  if(abs(accY) < accTotalVector){
    anglePitchAcc = asin((float)accY/accTotalVector)* 57.296;
  }
  if(abs(accX) < accTotalVector){
    angleRollAcc = asin((float)accX/accTotalVector)* -57.296;
  }
  
  //Place the MPU-6050 spirit level and note the values in the following two lines for calibration
  anglePitchAcc -= 0.0;  //Accelerometer calibration value for pitch
  angleRollAcc -= 0.0;  //Accelerometer calibration value for roll
  
  anglePitch = anglePitch * 0.9996 + anglePitchAcc * 0.0004;  //Correct the drift of the gyro pitch angle with the accelerometer pitch angle
  angleRoll = angleRoll * 0.9996 + angleRollAcc * 0.0004;  //Correct the drift of the gyro roll angle with the accelerometer roll angle

  pitchLevelpitchLevel = anglePitch * 15;
  rollLevel = angleRoll * 15;

  if(!autoLevel){
    pitchLevel = 0;
    rollLevel = 0;
  }


  //For starting the motors. Throttle low and yaw left
  if(receiverInput3 < 1050 && receiverInput4 < 1050)start = 1;

  //When yaw stick is back in the center position start the motors
  if(start == 1 && receiverInput3 < 1050 && receiverInput4 > 1450){

    start = 2;

    anglePitch = anglePitchAcc;  // Do this operation when the quadcopter is started
    angleRoll = angleRollAcc;  // Do this operation when the quadcopter is started
    gyroAngle = true;  //Set the IMU started flag

    //Reset the PID controllers for a bumpless start
    pidMemRoll = 0;
    pidMemPitch = 0;
    pidMemYaw = 0;

    pidErrorRoll = 0;
    pidErrorPitch = 0;
    pidErrorYaw = 0;
  }
  //Stopping the motors. throttle low and yaw right
  if(start == 2 && receiverInput3 < 1050 && receiverInput4 > 1950)start = 0;

  //The PID set point in degrees per second is determined by the roll receiver input
  pidSetpointRoll = 0;
  //Need a little dead band of 16us for better results
  if(receiverInput1 > 1508)pidSetpointRoll = receiverInput1 - 1508;
  else if(receiverInput1 < 1492)pidSetpointRoll = receiverInput1 - 1492;

  pidSetpointRoll -= rollLevel;
  pidSetpointRoll /= 3.0;

  pidSetpointPitch = 0;

  if(receiverInput2 > 1508)pidSetpointPitch = receiverInput2 - 1508;
  else if(receiverInput2 < 1492)pidSetpointPitch = receiverInput2 - 1492;

  pidSetpointPitch -= pitchLevel;
  pidSetpointPitch /= 3.0;

  pidSetpointYaw = 0;

  if(receiverInput3 > 1050){ //Don't yaw when turning off the motors
    if(receiverInput4 > 1508)pidSetpointYaw = (receiverInput4 - 1508)/3.0;
    else if(receiverInput4 < 1492)pidSetpointYaw = (receiverInput4 - 1492)/3.0;
  }
  
  pidCalculate();

  //The battery voltage is needed for compensation
  //A complementary filter is used to reduce noise
  batteryVoltage = batteryVoltage * 0.92 + (analogRead(0) + 65) * 0.09853;

  //Turn on the led if battery voltage is to low
  if(batteryVoltage < 1000 && batteryVoltage > 600)digitalWrite(12, HIGH);


  throttle = receiverInput3;  //Need the throttle signal as a base signal

  //The motors are started
  if (start == 2){
    if (throttle > 1800) throttle = 1800;

    esc1 = throttle - pidOutputPitch + pidOutputRoll - pidOutputYaw; //Calculate esc 1 (front-right - CCW)
    esc2 = throttle + pidOutputPitch + pidOutputRoll + pidOutputYaw; //Calculate esc 2 (rear-right - CW)
    esc3 = throttle + pidOutputPitch - pidOutputRoll - pidOutputYaw; //Calculate esc 3 (rear-left - CCW)
    esc4 = throttle - pidOutputPitch - pidOutputRoll + pidOutputYaw; //Calculate esc 4 (front-left - CW)

    if (batteryVoltage < 1240 && batteryVoltage > 800){  //Is the battery connected?
      //Compensate the esc1, esc2, esc3, esc4 pulse for voltage drop.
      esc1 += esc1 * ((1240 - batteryVoltage)/(float)3500);
      esc2 += esc2 * ((1240 - batteryVoltage)/(float)3500);
      esc3 += esc3 * ((1240 - batteryVoltage)/(float)3500);
      esc4 += esc4 * ((1240 - batteryVoltage)/(float)3500);
    } 

    if (esc1 < 1100) esc1 = 1100;
    if (esc2 < 1100) esc2 = 1100;
    if (esc3 < 1100) esc3 = 1100;
    if (esc4 < 1100) esc4 = 1100;

    if (esc1 > 2000) esc1 = 2000;
    if (esc2 > 2000) esc2 = 2000;
    if (esc3 > 2000) esc3 = 2000;
    if (esc4 > 2000) esc4 = 2000;
  }

  else{
    esc1 = 1000;
    esc2 = 1000;
    esc3 = 1000;
    esc4 = 1000;
  }

    
  if(micros() - loopTimer > 4050)digitalWrite(12, HIGH);
  
  //All the information for controlling the motor's is available
  while(micros() - loopTimer < 4000);
  loopTimer = micros();

  PORTD |= B11110000;  //Set digital outputs 4,5,6 and 7 high

  //Calculate the time of the faling edge of the esc1, esc2, esc3, esc4 pulse
  timerChannel1 = esc1 + loopTimer;  
  timerChannel2 = esc2 + loopTimer;
  timerChannel3 = esc3 + loopTimer;
  timerChannel4 = esc4 + loopTimer;
  
  //There is always 1000us of spare time. So let's do something usefull that is very time consuming
  //Get the current gyro and receiver data and scale it to degrees per second for the pid calculations
  gyroSignal();

  while(PORTD >= 16){  //Stay in this loop until output 4,5,6 and 7 are low
    escTimerLoop = micros();  //Read the current time
    //Set digital output 4, 5, 6, 7 to low if the time is expired
    if(timerChannel1 <= escTimerLoop)PORTD &= B11101111;
    if(timerChannel2 <= escTimerLoop)PORTD &= B11011111;
    if(timerChannel3 <= escTimerLoop)PORTD &= B10111111;
    if(timerChannel4 <= escTimerLoop)PORTD &= B01111111;
  }
}

//This routine is called every time input 8, 9, 10 or 11 changed state. This is used to read the receiver signals
ISR(PCINT0_vect){
  currentTime = micros();
  //Channel 1
  if(PINB & B00000001){  //Is input 8 high?
    if(lastChannel == 0){  //Input 8 changed from 0 to 1
      lastChannel = 1;
      timer1 = currentTime;
    }
  }
  else if(lastChannel == 1){  //Input 8 is not high and changed from 1 to 0
    lastChannel = 0;
    receiverInput[1] = currentTime - timer1;
  }
  //Channel 2
  if(PINB & B00000010 ){  //Is input 9 high?
    if(lastChannel2 == 0){  //Input 9 changed from 0 to 1
      lastChannel2 = 1;
      timer2 = currentTime;
    }
  }
  else if(lastChannel2 == 1){  //Input 9 is not high and changed from 1 to 0
    lastChannel2 = 0;
    receiverInput[2] = currentTime - timer2;
  }
  //Channel 3
  if(PINB & B00000100 ){  //Is input 10 high?
    if(lastChannel3 == 0){  //Input 10 changed from 0 to 1
      lastChannel3 = 1;
      timer3 = currentTime;
    }
  }
  else if(lastChannel3 == 1){  //Input 10 is not high and changed from 1 to 0
    lastChannel3 = 0;
    receiverInput[3] = currentTime - timer3;

  }
  //Channel 4
  if(PINB & B00001000 ){  //Is input 11 high?
    if(lastChannel4 == 0){  //Input 11 changed from 0 to 1
      lastChannel4 = 1;
      timer4 = currentTime;
    }
  }
  else if(lastChannel4 == 1){  //Input 11 is not high and changed from 1 to 0
    lastChannel4 = 0;
    receiverInput[4] = currentTime - timer4;
  }
}


//Reading the gyro
void gyroSignal(){
  //Read the MPU-6050
  if(eepromData[31] == 1){
    Wire.beginTransmission(gyroAddress);
    Wire.write(0x3B);
    Wire.endTransmission();
    Wire.requestFrom(gyroAddress,14);
    
    receiverInput1 = convertReceiverChannel(1);  //Convert the actual receiver signals for pitch to the standard 1000 - 2000us
    receiverInput2 = convertReceiverChannel(2);  //Convert the actual receiver signals for roll to the standard 1000 - 2000us
    receiverInput3 = convertReceiverChannel(3);  //Convert the actual receiver signals for throttle to the standard 1000 - 2000us
    receiverInput4 = convertReceiverChannel(4);  //Convert the actual receiver signals for yaw to the standard 1000 - 2000us
    
    while(Wire.available() < 14);
    //Add the low and high byte to the accX, accY, accZ variables
    accAxis[1] = Wire.read()<<8|Wire.read();  
    accAxis[2] = Wire.read()<<8|Wire.read();
    accAxis[3] = Wire.read()<<8|Wire.read();

    temperature = Wire.read()<<8|Wire.read();  //Add the low and high byte to the temperature variable

    //Read high and low part of the angular data
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
  gyroRoll = gyroAxis[eepromData[28] & 0b00000011];
  if(eepromData[28] & 0b10000000)gyroRoll *= -1;
  gyroPitch = gyroAxis[eepromData[29] & 0b00000011];
  if(eepromData[29] & 0b10000000)gyroPitch *= -1;
  gyroYaw = gyroAxis[eepromData[30] & 0b00000011];
  if(eepromData[30] & 0b10000000)gyroYaw *= -1;

  accX = accAxis[eepromData[29] & 0b00000011];
  if(eepromData[29] & 0b10000000)accX *= -1;
  accY = accAxis[eepromData[28] & 0b00000011];
  if(eepromData[28] & 0b10000000)accY *= -1;
  accZ = accAxis[eepromData[30] & 0b00000011];
  if(eepromData[30] & 0b10000000)accZ *= -1;
}


void pidCalculate(){

  //Roll calculations
  pidError = gyroInputRoll - pidSetpointRoll;
  pidMemRoll += pidGainRollForIController * pidError;
  if(pidMemRoll > pidRollMax)pidMemRoll = pidRollMax;
  else if(pidMemRoll < pidRollMax * -1)pidMemRoll = pidRollMax * -1;

  pidOutputRoll = pidGainRollForPController * pidError + pidMemRoll + pidGainRollForDController * (pidError - pidErrorRoll);
  if(pidOutputRoll > pidRollMax)pidOutputRoll = pidRollMax;
  else if(pidOutputRoll < pidRollMax * -1)pidOutputRoll = pidRollMax * -1;

  pidErrorRoll = pidError;

  //Pitch calculations
  pidError = gyroInputPitch - pidSetpointPitch;
  pidMemPitch += pidGainPitchForIController * pidError;
  if(pidMemPitch > pidPitchMax)pidMemPitch = pidPitchMax;
  else if(pidMemPitch < pidPitchMax * -1)pidMemPitch = pidPitchMax * -1;

  pidOutputPitch = pidGainPitchForPController * pidError + pidMemPitch + pidGainPitchForDController * (pidError - pidErrorPitch);
  if(pidOutputPitch > pidPitchMax)pidOutputPitch = pidPitchMax;
  else if(pidOutputPitch < pidPitchMax * -1)pidOutputPitch = pidPitchMax * -1;

  pidErrorPitch = pidError;

  //Yaw calculations
  pidError = gyroInputYaw - pidSetpointYaw;
  pidMemYaw += pidGainYawForIController * pidError;
  if(pidMemYaw > pidYawMax)pidMemYaw = pidYawMax;
  else if(pidMemYaw < pidYawMax * -1)pidMemYaw = pidYawMax * -1;

  pidOutputYaw = pidGainYawForPController * pidError + pidMemYaw + pidGainYawForDController * (pidError - pidErrorYaw);
  if(pidOutputYaw > pidYawMax)pidOutputYaw = pidYawMax;
  else if(pidOutputYaw < pidYawMax * -1)pidOutputYaw = pidYawMax * -1;

  pidErrorYaw = pidError;
}

//This part converts the actual receiver signals to a standardized 1000 – 1500 – 2000 microsecond value.
//The stored data in the EEPROM is used.
int convertReceiverChannel(byte function){
  int low, center, high, actual;
  int difference;
  byte channel, reverse;

  channel = eepromData[function + 23] & 0b00000111;  //What channel corresponds with the specific function
  if(eepromData[function + 23] & 0b10000000)reverse = 1;  //Reverse channel when most significant bit is set
  else reverse = 0;  //If the most significant is not set there is no reverse

  actual = receiverInput[channel];  //Read the actual receiver value for the corresponding function

  //Store the low, center, high values for the specific receiver input channel
  low = (eepromData[channel * 2 + 15] << 8) | eepromData[channel * 2 + 14];  
  center = (eepromData[channel * 2 - 1] << 8) | eepromData[channel * 2 - 2];
  high = (eepromData[channel * 2 + 7] << 8) | eepromData[channel * 2 + 6];

  if(actual < center){
    if(actual < low)actual = low;  //Limit the lowest value to the value that was detected during setup
    difference = ((long)(center - actual) * (long)500) / (center - low);  //Calculate and scale the actual value to a 1000 - 2000us value
    if(reverse == 1)return 1500 + difference;  //If the channel is reversed
    else return 1500 - difference;  //If the channel is not reversed
  } 
  else if(actual > center){
    if(actual > high)actual = high;  //Limit the lowest value to the value that was detected during setup
    difference = ((long)(actual - center) * (long)500) / (high - center);  //Calculate and scale the actual value to a 1000 - 2000us value
    if(reverse == 1)return 1500 - difference;  //If the channel is reversed
    else return 1500 + difference;  //If the channel is not reversed
  }
  else return 1500;
}

void setGyroRegisters(){
  //Setup the MPU-6050
  if(eepromData[31] == 1){
    Wire.beginTransmission(gyroAddress);  //Start communication with the address found during search.
    Wire.write(0x6B);
    Wire.write(0x00);
    Wire.endTransmission();

    Wire.beginTransmission(gyroAddress);
    Wire.write(0x1B);
    Wire.write(0x08);
    Wire.endTransmission();

    Wire.beginTransmission(gyroAddress);
    Wire.write(0x1C);
    Wire.write(0x10);
    Wire.endTransmission();

    //Perform a random register check to see if the values are correct
    Wire.beginTransmission(gyroAddress);
    Wire.write(0x1B);
    Wire.endTransmission();
    Wire.requestFrom(gyroAddress, 1);  //Request 1 bytes from the gyro
    while(Wire.available() < 1);
    if(Wire.read() != 0x08){
      digitalWrite(12,HIGH);  //Turn on the warning led
      while(1)delay(10);  //Stay in this loop forever
    }

    Wire.beginTransmission(gyroAddress);
    Wire.write(0x1A);
    Wire.write(0x03);
    Wire.endTransmission(); 

  }  
}
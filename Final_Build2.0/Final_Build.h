#include <Wire.h>
#include <SoftwareSerial.h>
#include <Adafruit_PWMServoDriver.h>
#include <TCA9555.h>

///        I2C ADDRESSING BLOCK       ////
TCA9555 tca9555(0,0,0);
// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
// you can also call it with a different address you want
// Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);

/////             GLOBALS              /////
#define SERVOMIN  120 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  560 // this is the 'maximum' pulse length count (out of 4096)
uint8_t servonum = 0;                  // our servo # counter
byte outputState;                      // our LED register
int motorArray = [3,5,6,9];            // array holding motor pin numbers

//////       COMMAND FUNCTIONS       ////////

// Command "M" = Motor --- Format = "MC###" five char command where: 
//   C = [A = all motors, F = front, B = back, L = left, R = right, 0-3 = motors 0-3]
// ### = value between 40(slowest)-200(fastest) 
void motorSet(String fiveDig)
{  
  int commandArray = [0,0,0,0];  // keeps track of what motors to act upon
  String value = fiveDig.substring(1);  // value to set motors (use 50 to 200)
  
  // Switch case to check which motors to act upon
    switch (fiveDig.charAt(0))
    {
      case '0':
        commandArray = [1,0,0,0];
        break;
      case '1':
        commandArray = [0,1,0,0];
        break;
      case '2':
        commandArray = [0,0,1,0];
        break;
      case '3': 
        commandArray = [0,0,0,1];
        break; 
      case 'A': // All motors
        commandArray = [1,1,1,1];
        break;
      case 'L': // Left motors
        commandArray = [1,0,1,0];
        break;
      case 'R':// Right motors
        commandArray = [0,1,0,1];
        break;
    }

    
    // PRINT CHECK
    Serial.println("Motor Set");
    Serial.println("commandArray = ");
    Serial.print(commandArray);
    Serial.println("value = ");
    Serial.print(value);

    // For each motor in array, set value
    for (j = 0; j < 4; j++)
    {
      if (commandArray[j])
        analogWrite(motorArray,value.toInt());
    }
    return;
}

// Command "S" = Servo --- Format = "SC###" five char command where:
//   C = [0 = servo 0, 1 = servo1]
// ### = value between 0(raised all the way)-100(lowered all the way)
void dirSet(String twoDig)
{
  String firstChar = twoDig.charAt(0);
  int pinNum = firstChar.toInt();

  //PRINT CHECK
  Serial.println("first Char = ");
  Serial.print(firstChar);
  Serial.println("pinNum = ");
  Serial.println(pinNum);
  
  if (twoDig.charAt(1) == 'F')                 // If forward
    outputState = tca9555.setOutON(pinNum, outputState);
  if (twoDig.charAt(1) == 'R')                 // If reverse
    outputState = tca9555.setOutOFF(pinNum, outputState);

  tca9555.setOutputStates(pinNum, outputState); // set extender to reflect changes
  return;
}

// Command "D" = Motor Direction Set --- Format = "D#C" three char command where:
//   # = motor# to set (between 0-3)
//   C = motor direction [F = forward, R = Reverse]
void servoSet(String fourDig)
{
  String firstChar = fourDig.charAt(0);
  int servonum = firstChar.toInt();
  String commandPreset = fourDig.substring(1);
  int commandValue = commandPreset.toInt();
  commandValue = commandValue(440/100)+120;
  
  // PRINT CHECK
  Serial.println("first Char = ");
  Serial.print(firstChar);
  Serial.println("commandPreset = ");
  Serial.print(commandPreset);
  Serial.println("commandValue = ");
  Serial.print(commandValue);

  if (commandPreset == "MAX")
  {
    for (uint16_t pulselen = SERVOMIN; pulselen < SERVOMAX; pulselen++) {
        pwm.setPWM(servonum, 0, pulselen);
        }  
  }
  if (commandPreset == "MIN")
  {
    for (uint16_t pulselen = SERVOMAX; pulselen > SERVOMIN; pulselen--) {
        pwm.setPWM(servonum, 0, pulselen);
        }  
  }
  else 
  {
      uint16_t pulselen = commandValue;
      pwm.setPWM(servonum, 0, pulselen);
      pwm.setPWM(servonum, 0, pulselen);
      pwm.setPWM(servonum, 0, pulselen);
  }
  return;
}

// Command "L" = LED Set --- Format = "LN#" three char command where:
//   N = LED# to set (between 0-7)
//   # = ON/OFF [0 = off, 1 = on]
void LEDSet(String twoDig)
{
  String firstDig = twoDig.charAt(0);
  int pinNum = firstDig.toInt();
  if (twoDig.substring(1) == "0")
    pwm.setPin(pinNum+5,0);
  if (twoDig.substring(1) == "1")
    pwm.setPin(pinNum+5,4095);
  return;
}
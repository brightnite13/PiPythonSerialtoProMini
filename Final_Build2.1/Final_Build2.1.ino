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
int motorArray[4] = {3, 5, 6, 9};            // array holding motor pin numbers

void setup() {
  Serial.begin(115200);
  pinMode(A0,INPUT);
  pinMode(A1,INPUT);
  pinMode(3,OUTPUT);
  pinMode(5,OUTPUT);
  pinMode(6,OUTPUT);
  pinMode(9,OUTPUT);
  Serial.println("Arduino Pro Mini Serial Command Handler");
  Wire.begin();
  outputState = 0x0f;
  
  //set the pin directions for ports to output
  tca9555.setPortDirection(TCA9555::DIR_OUTPUT);
  #ifdef ESP8266
  Wire.pins(2, 14);   // ESP8266 can use any two pins, such as SDA to #2 and SCL to #14
  #endif
  
  pwm.begin();
  pwm.setPWMFreq(60);  // This is the maximum PWM frequency

  // if you want to really speed stuff up, you can go into 'fast 400khz I2C' mode
  // some i2c devices dont like this so much so if you're sharing the bus, watch
  // out for this!
  #ifdef TWBR    
  // save I2C bitrate
  uint8_t twbrbackup = TWBR;
  // must be changed after calling Wire.begin() (inside pwm.begin())
  TWBR = 12; // upgrade to 400KHz!
  #endif
  
  yield(); // Needed for servos to funtion
  
}

void printInput()
{

      Serial.println("Analog input A0");
      Serial.println(analogRead(A0));
      
      Serial.println("Analog input A1");
      Serial.println(analogRead(A1));
      
    
  
  return;  
}


void motorSet(String fiveDig)
{  
  int commandArray[4] = {0,0,0,0};  // keeps track of what motors to act upon
  String value = fiveDig.substring(1,5);  // value to set motors (use 50 to 200)
  char swCase = fiveDig.charAt(0);
  // Switch case to check which motors to act upon
    switch (swCase)
    {
      case '0':
        commandArray[0] = 1;
        break;
      case '1':
        commandArray[1] = 1;
        break;
      case '2':
        commandArray[2] = 1;
        break;
      case '3': 
        commandArray[3] = 1;
        break; 
      case 'A': // All motors
        for (int k = 0;k<4;k++)
          commandArray[k] = 1;
        break;
      case 'L': // Left motors
        commandArray[0] = 1;
        commandArray[2] = 1;
        break;
      case 'R':// Right motors
        commandArray[1] = 1;
        commandArray[3] = 1;
        break;
    }

    
    // PRINT CHECK
    Serial.println("Motor Set");
    Serial.println("commandArray = ");
    Serial.println("value = ");
    Serial.println(value);

    // For each motor in array, set value
    for (int j = 0; j < 4; j++)
    {
      if (commandArray[j])
        analogWrite(motorArray[j],value.toInt());
    }
    return;
}

void delaySet(String threeChar)
{
  delay(threeChar.toInt());
  return;
}

void dirSet(String twoDig)
{
  String firstChar = twoDig.substring(0,1);
  int pinNum = firstChar.toInt();
  String secondChar = twoDig.substring(1,2);
  

  //PRINT CHECK
  Serial.println("first Char = ");
  Serial.println(firstChar);
  Serial.println("pinNum = ");
  Serial.println(pinNum);
  Serial.println("secondChar = ");
  Serial.println(secondChar);
  if (firstChar=="A")
  {     
    if (twoDig.substring(1,2) == "R")                 // If forward
      outputState = tca9555.setOutON(0, outputState);
    if (twoDig.substring(1,2) == "F")                 // If reverse
      outputState = tca9555.setOutOFF(0, outputState);
  
    if (twoDig.substring(1,2) == "R")                 // If forward
      outputState = tca9555.setOutON(1, outputState);
    if (twoDig.substring(1,2) == "F")                 // If reverse
    outputState = tca9555.setOutOFF(1, outputState);
    tca9555.setOutputStates(1, outputState); // set extender to reflect changes
    tca9555.setOutputStates(0, outputState); // set extender to reflect changes 
  }
  else
  {
    if (twoDig.substring(1,2) == "R")                 // If forward
      outputState = tca9555.setOutON(pinNum, outputState);
    if (twoDig.substring(1,2) == "F")                 // If reverse
      outputState = tca9555.setOutOFF(pinNum, outputState);  
    tca9555.setOutputStates(pinNum, outputState); // set extender to reflect changes
  }
  return;
}

void servoSet(String fourDig)
{
  String firstChar = fourDig.substring(0,1);
  int servonum = firstChar.toInt();
  String commandPreset = fourDig.substring(1);
  int commandValue = commandPreset.toInt();
  commandValue = (commandValue*(440/100))+120;
  
  // PRINT CHECK
  Serial.println("first Char = ");
  Serial.println(firstChar);
  Serial.println("commandPreset = ");
  Serial.println(commandPreset);
  Serial.println("commandValue = ");
  Serial.println(commandValue);
  Serial.println("servoNum = ");
  Serial.println(servonum);

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

void LEDSet(String twoDig)
{
  String firstDig = twoDig.substring(0,1);
  int pinNum = firstDig.toInt();
  if (twoDig.substring(1) == "0")
    pwm.setPin(pinNum+5,0);
  if (twoDig.substring(1) == "1")
    pwm.setPin(pinNum+5,4095);
  return;
}

/////// Loop scans inputString for command variables. Format sequentially with no spaces////////
// Command "M" = Motor --- Format = "MC###" five char command where: 
//   C = [A = all motors, F = front, B = back, L = left, R = right, 0-3 = motors 0-3]
// ### = value between 40(slowest)-200(fastest)
// 
// Command "S" = Servo --- Format = "SC###" five char command where:
//   C = [0 = servo 0, 1 = servo1]
// ### = value between 0(raised all the way)-100(lowered all the way)
//     
// Command "D" = Motor Direction Set --- Format = "D#C" three char command where:
//   # = motor# to set (between 0-3)
//   C = motor direction [F = forward, R = Reverse]
//
// Command "L" = LED Set --- Format = "LN#" three char command where:
//   N = LED# to set (between 0-7)
//   # = ON/OFF [0 = off, 1 = on]
///////////////////////////////////////////////////////////////////////////////////////////////////

void loop() {
  String incomingString;
  
  if(Serial.available()>0){  // loop to monitor and read serial commands
    incomingString = Serial.readString();  // string to hold incoming serial data
    
    // Checks string for Command Variable and executes if found
    for(int i = 0;i < incomingString.length();i++)
    {
      //if (incomingString.substring(i,i+1) == "W"){
        //delaySet(incomingString.substring(i+1,i+4));
        //Serial.println("sentString = ");
        //Serial.println(incomingString.substring(i+1,i+4));
      //}
      if (incomingString.substring(i,i+1) == "M"){
        motorSet(incomingString.substring(i+1,i+5));
        Serial.println("sentString = ");
        Serial.println(incomingString.substring(i+1,i+5));
      }
      if (incomingString.substring(i,i+1) == "S"){
        servoSet(incomingString.substring(i+1,i+5));
        Serial.println("sentString = ");
        Serial.println(incomingString.substring(i+1,i+5));
      }
      if (incomingString.substring(i,i+1) == "L"){
        LEDSet(incomingString.substring(i+1,i+2));
        Serial.println("sentString = ");
        Serial.println(incomingString.substring(i+1,i+2));
      }
      if (incomingString.substring(i,i+1) == "D"){
        dirSet(incomingString.substring(i+1,i+3));
        Serial.println("sentString = ");
        Serial.println(incomingString.substring(i+1,i+3));
      }
    }
  Serial.println(incomingString);
  }
  printInput();
}

void initialize()
{
  dirSet("0F");//full speed forward
  dirSet("1F");
  motorSet("A200");
  delay(500);
  motorSet("A060");// slow forward
  delay(1000);  
  motorSet("A000"); //off
  delay(1000);
  servoSet("0050");// move servos
  servoSet("1070");
  delay(1000);
  dirSet("0F");// turn right
  dirSet("1R");
  motorSet("L200");
  motorSet("R200");
  delay(1000); 
  dirSet("0R");// turn left
  dirSet("1F");
  motorSet("L100");
  motorSet("R100");

  servoSet("0100");
  servoSet("1100");
  delay(1000);
  dirSet("0F");
  dirSet("1R");
  motorSet("L100");
  motorSet("R100");

  servoSet("0100");
  servoSet("1100");
  delay(100);
  motorSet("A000");
  return;
}



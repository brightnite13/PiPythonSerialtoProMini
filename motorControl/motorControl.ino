import time
import Serial

////////////////         GLOBALS         /////////////////
#define maxMotorPower = 90;
#define minMotorPower = 50;
signed float calibrationConstant;
float motorSpeed;


void setup() {
  // initialize serial connection
  Serial.begin(115200);
  // set the motor pins
  pinMode(3,OUTPUT);
  pinMode(5,OUTPUT;
  // initialize global constants
  calibrationConstant = 0;
  minMotorPower = 45;
  maxMotorPower = 90;
}

void autoForward()
{
  dirSet("0F");
  dirSet("1F");
  motorOn();
  // if !pollCheckLeft then correct left
  // if !pollCheckRight then correct right
  // if !pollcheckForward then stop
  return;  
}

void autoBackward()
{
  dirSet("0R");
  dirSet("1R");
  motorOn();
  // if !pollCheckLeft then correct left
  // if !pollCheckRight then correct right
  // if !pollcheckBack then stop
  return;  
}

void 

////////////////         FUNCTIONS         /////////////////
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
// IN:  Takes in a five digit string formatted for zero point (-like) rotation
// DESC: 
void rotation(String fiveDig)
{
  if (fiveDig.subString(0,1) == "L")
  {
    dirSet("0R");
    dirSet("1F"); 
  }
  if (fiveDig.subString(0,1) == "R")
  {
    dirSet("0F");
    dirSet("1R");
  }
  motorOn(75);
  delay(fiveDig.substring(1).toInt());
  motorOff();
  return;
}

void motorOff()
{
  analogWrite(3,0);
  analogWrite(5,0);
  Serial.print("Motors off");
  return;  
}

void motorOn(float mSpeed)
{
  analogWrite(3, mSpeed+calibrationConstant);
  analogWrite(5, mSpeed);    
  Serial.print("Left Speed = ",mSpeed+calibrationConstant);
  Serial.print("Right Speed = ",mSpeed);
  return;
}

// IN:  Takes in a five digit string formatted for linear forward/backward motion
// DESC:
void linearMotion(String fiveDig)
{
  // Set direction
  if (fiveDig.subString(0,1) == "F")
  {
    dirSet("0F");
    dirSet("1F");

  }
  if (fiveDig.subString(0,1) == "R")
  {
    dirSet("0R");
    dirSet("1R");
  }
  // Set speed 
  analogWrite(3, fiveDig.substring(1).toInt()+calibrationConstant);
  analogWrite(5, fiveDig.substring(1).toInt());    
  Serial.print("Left Speed = ",fiveDig.substring(1).toInt()+calibrationConstant);
  Serial.print("Right Speed = ",fiveDig.substring(1).toInt());
  return;
}


// IN:  Calibrates the fixed motor speed for straight line movement
// DESC: 
void motorCalibration(){  
  while 1{  
    float rSpeed = 75;
    float lSpeed = 75;
    if(Serial.available()>0)
    {  
      incomingString = Serial.readString();
      for (int k = 0; k < incomingString.length(); k++)
      {
        if (incomingString.substring(k,k+1) == "Q")  //If "Q" then quit back to main loop
        {
          Serial.print("Now Quitting Calibration");
          return; 
        }

        if (incomingString.substring(k,k+1) == "O") //If "O" then increment calibration by 2.5
        {
          calibrationConstant += 2.5;
          Serial.print(calibrationConstant);
        }
        if (incomingString.substring(k,k+1) == "L") //If "L" then decrement calibration by 2.5
        {
          calibrationConstant += 2.5;
          Serial.print(calibrationConstant);
        }
        if (incomingString.substring(k,k+1) == "I") //If "I" then manual increment by three digit float to the tenth ##.# (do not include decimal)
        {
          calibrationConstant += incomingString.substring(k+1,k+4).toInt() / 10; 
          Serial.print(calibrationConstant);
        }
        if (incomingString.substring(k,k+1) == "K") //If "K" manual decrement by three digit float to the tenth
        {
          calibrationConstant -= incomingString.substring(k+1,k+4).toInt() / 10;
          Serial.print(calibrationConstant);
        }
      }
      // print new speeds
      Serial.print("Pin 3 at speed: ", (lspeed + calibrationConstant));
      Serial.print("Pin 5 at speed: ", rspeed);
      Serial.print("Calibration Constant Set To: ", calibrationConstant);
    }  
    // adjust to new speed 
    analogWrite(3, lspeed + calibrationConstant);
    analogWrite(5, rSpeed);
  }
}

void loop() 
{
 String incomingString;
  if(Serial.available()>0){  // loop to monitor and read serial commands
    incomingString = Serial.readString();  // string to hold incoming serial data
    for (int i = 0; i < incomingString.length(); i++)
    {
      if (incomingString.substring(i,i+1) == "C") //If "C" run Motor Speed Calibration
      {
        Serial.print("Begin Motor Callibration");
        motorCalibration();
      }
      if (incomingString.substring(i,i+1) == "V") //If "V" run Wall Following mode   
      {
        Serial.print("Begin Wall Following");
        //wallFollow();
      }
      if (incomingString.substring(i,i+1) == "W") //If "W" begin forward motion
      {
        String temp = "F";
        linearMotion(temp + incomingString.substring(i+1,i+4);
      }
      if (incomingString.substring(i,i+1) == "S") //If "S" begin reverse motion
      {
        String temp = "R";
        linearMotion(temp + incomingString.substring(i+1,i+4);
      }
      if (incomingString.substring(i,i+1) == "A") //If "A" begin left turn
      {
        String temp = "L";
        rotation(temp + incomingString.substring(i+1,i+4);
      }
      if (incomingString.substring(i,i+1) == "D") //If "A" begin left turn
      {
        String temp = "R";
        rotation(temp + incomingString.substring(i+1,i+4);
      }
    }
  }
}

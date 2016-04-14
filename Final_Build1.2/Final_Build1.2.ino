
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <TCA9555.h>


///        I2C ADDRESSING BLOCK       ////
// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
// you can also call it with a different address you want
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);

// Depending on your servo make, the pulse width min and max may vary, you 
// want these to be as small/large as possible without hitting the hard stop
// for max range. You'll have to tweak them as necessary to match the servos you
// have!
#define SERVOMIN  120 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  560 // this is the 'maximum' pulse length count (out of 4096)

// our servo # counter
uint8_t servonum = 0;
#include <SoftwareSerial.h>

TCA9555 tca9555(0,0,0);

String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete
byte outputState;

void setup() {
  Serial.begin(9600);
  pinMode(3,OUTPUT);
  pinMode(5,OUTPUT);
  pinMode(6,OUTPUT);
  pinMode(9,OUTPUT);
  Serial.println("Arduino Pro Mini Serial Command Handler");
  Wire.begin();
  outputState = 0x0f;
  
  //set the pin directions for both ports to 
  //output
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
  yield();
}


// you can use this function if you'd like to set the pulse length in seconds
// e.g. setServoPulse(0, 0.001) is a ~1 millisecond pulse width. its not precise!
void setServoPulse(uint8_t n, double pulse) {
  double pulselength;
  
  pulselength = 1000000;   // 1,000,000 us per second
  pulselength /= 60;   // 60 Hz
  Serial.print(pulselength); Serial.println(" us per period"); 
  pulselength /= 4096;  // 12 bits of resolution
  Serial.print(pulselength); Serial.println(" us per bit"); 
  pulse *= 1000;
  pulse /= pulselength;
  Serial.println(pulse);
  pwm.setPWM(n, 0, pulse);
}

void loop() {
  String incomingString;

  if(Serial.available()>0){  // loop to monitor and read serial commands
    incomingString = Serial.readString();  // string to hold incoming serial data
    Serial.println(incomingString.substring(0,9));

//////////////////////////////////////////////////////////////
//////////////////BEGIN; COMMANDS TO ARDUINO /////////////////
//////////////////////////////////////////////////////////////


     // MOTOR 0 ON
    if (incomingString == "MOTOR0ON"){
      Serial.println("Motor 0 on");
      digitalWrite(3,HIGH);
    }
    // MOTOR 0 OFF
    if (incomingString == "MOTOR0OFF"){
      Serial.println("Motor 0 off");
      analogWrite(3,0);
    }       
    // MOTOR 0 SET    
    if (incomingString.substring(0,9) == "MOTOR0SET"){
      Serial.println("Motor 0 Set");
      Serial.println(incomingString);
      String value= incomingString.substring(9,12);
      Serial.println(value);
      analogWrite(3,value.toInt());
      Serial.println(value);
    }

     // MOTOR 1 ON
    if (incomingString == "MOTOR1ON"){
      Serial.println("Motor 1 on");
      digitalWrite(5,HIGH);
    }
    // MOTOR 1 OFF
    if (incomingString == "MOTOR0OFF"){
      Serial.println("Motor 1 off");
      analogWrite(5,0);
    }
    // MOTOR 1 SET    
    if (incomingString.substring(0,9) == "MOTOR1SET"){
      Serial.println("Motor 1 Set");
      Serial.println(incomingString);
      String value= incomingString.substring(9,12);
      Serial.println(value);
      analogWrite(5,value.toInt());
      Serial.println(value);
    }
    
     // MOTOR 2 ON
    if (incomingString == "MOTOR2ON"){
      Serial.println("Motor 0 on");
      digitalWrite(6,HIGH);
    }
    // MOTOR 2 OFF
    if (incomingString == "MOTOR2OFF"){
      Serial.println("Motor 0 off");
      analogWrite(6,0);
    }
    // MOTOR 2 SET    
    if (incomingString.substring(0,9) == "MOTOR2SET"){
      Serial.println("Motor 0 Set");
      Serial.println(incomingString);
      String value= incomingString.substring(9,12);
      Serial.println(value);
      analogWrite(6,value.toInt());
      Serial.println(value);
    }
    
    // MOTOR 3 ON
    if (incomingString == "MOTOR3ON"){
      Serial.println("Motor 0 on");
      digitalWrite(9,HIGH);
    }
    // MOTOR 3 OFF
    if (incomingString == "MOTOR3OFF"){
      Serial.println("Motor 0 off");
      analogWrite(9,0);
    }
    // MOTOR 3 SET    
    if (incomingString.substring(0,9) == "MOTOR3SET"){
      Serial.println("Motor 3 Set");
      Serial.println(incomingString);
      String value= incomingString.substring(9,12);
      Serial.println(value);
      analogWrite(9,value.toInt());
      Serial.println(value);
    }

//////////////////////////////////////////////////////////////
///////////////////END; COMMANDS TO ARDUINO///////////////////
//////////////////////////////////////////////////////////////

    
//////////////////////////////////////////////////////////////
//////////////////BEGIN; COMMANDS TO 9554a ///////////////////
//////////////////////////////////////////////////////////////
    
    // EXTENDER PIN 0 ON
    if (incomingString == "EXTENDER0ON"){
      Serial.println("Extender pin 0 on");
      outputState = tca9555.setOutON(0, outputState);  // calculate new value for output register
      tca9555.setOutputStates(0, outputState);  // set output register to new value
    }
    // EXTENDER PIN 0 OFF
    if (incomingString == "EXTENDER0OFF"){
      Serial.println("Extender pin 0 off");
      outputState = tca9555.setOutOFF(0, outputState);
      tca9555.setOutputStates(0, outputState);
    }
    // EXTENDER PIN 1 ON
    if (incomingString == "EXTENDER1ON"){
      Serial.println("Extender pin 1 on");
      outputState = tca9555.setOutON(1, outputState);
      tca9555.setOutputStates(0, outputState);
    }
    // EXTENDER PIN 1 OFF
    if (incomingString == "EXTENDER1OFF"){
      Serial.println("Extender pin 1 off");
      outputState = tca9555.setOutOFF(1, outputState);
      tca9555.setOutputStates(0, outputState);
    }
    // EXTENDER PIN 2 ON
    if (incomingString == "EXTENDER2ON"){
      Serial.println("Extender pin 2 on");
      outputState = tca9555.setOutON(2, outputState);
      tca9555.setOutputStates(0, outputState);
    }
    // EXTENDER PIN 2 OFF
    if (incomingString == "EXTENDER2OFF"){
      Serial.println("Extender pin 2 off");
      outputState = tca9555.setOutOFF(2, outputState);
      tca9555.setOutputStates(0, outputState);
    }
    // EXTENDER PIN 3 ON
    if (incomingString == "EXTENDER3ON"){
      Serial.println("Extender pin 3 on");
      outputState = tca9555.setOutON(3, outputState);
      tca9555.setOutputStates(0, outputState);
    }
    // EXTENDER PIN 3 OFF
    if (incomingString == "EXTENDER3OFF"){
      Serial.println("Extender pin 3 off");
      outputState = tca9555.setOutOFF(3, outputState);
      tca9555.setOutputStates(0, outputState);
    }
    // EXTENDER PIN 4 ON
    if (incomingString == "EXTENDER4ON"){
      Serial.println("Extender pin 4 on");
      outputState = tca9555.setOutON(4, outputState);
      tca9555.setOutputStates(0, outputState);
    }
    // EXTENDER PIN 4 OFF
    if (incomingString == "EXTENDER4OFF"){
      Serial.println("Extender pin 4 off");
      outputState = tca9555.setOutOFF(4, outputState);
      tca9555.setOutputStates(0, outputState);
    }
    // EXTENDER PIN 5 ON
    if (incomingString == "EXTENDER5ON"){
      Serial.println("Extender pin 5 on");
      outputState = tca9555.setOutON(5, outputState);
      tca9555.setOutputStates(0, outputState);
    }
    // EXTENDER PIN 5 OFF
    if (incomingString == "EXTENDER5OFF"){
      Serial.println("Extender pin 5 off");
      outputState = tca9555.setOutOFF(5, outputState);
      tca9555.setOutputStates(0, outputState);
    }
    // EXTENDER PIN 6 ON
    if (incomingString == "EXTENDER6ON"){
      Serial.println("Extender pin 6 on");
      outputState = tca9555.setOutON(6, outputState);
      tca9555.setOutputStates(0, outputState);
    }
    // EXTENDER PIN 6 OFF
    if (incomingString == "EXTENDER6OFF"){
      Serial.println("Extender pin 6 off");
      outputState = tca9555.setOutOFF(6, outputState);
      tca9555.setOutputStates(0, outputState);
    }
    // EXTENDER PIN 7 ON
    if (incomingString == "EXTENDER7ON"){
      Serial.println("Extender pin 7 on");
      outputState = tca9555.setOutON(7, outputState);
      tca9555.setOutputStates(0, outputState);
    }
    // EXTENDER PIN 7 OFF
    if (incomingString == "EXTENDER7OFF"){
      Serial.println("Extender pin 7 off");
      outputState = tca9555.setOutOFF(7, outputState);
      tca9555.setOutputStates(0, outputState);
    }
////////////////////////////////////////////////////////////
//////////////////END; COMMANDS TO 9554a ///////////////////
////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////
//////////////////BEGIN; COMMANDS TO 9655 //////////////////
////////////////////////////////////////////////////////////
    // SERVO 1 CONTROL TO MAX
    if (incomingString == "SERVO0MAX"){
      for (uint16_t pulselen = SERVOMIN; pulselen < SERVOMAX; pulselen++) {
        pwm.setPWM(servonum, 0, pulselen);
        }
    }

    // SERVO 1 CONTROL TO MIN
    if (incomingString == "SERVO0MIN"){
      for (uint16_t pulselen = SERVOMAX; pulselen > SERVOMIN; pulselen--) {
        pwm.setPWM(servonum, 0, pulselen);
        }
    }
    // SERVO 3 SETMAX    
    if (incomingString.substring(0,10) == "SERVO0SETF"){
      Serial.println("Servo 0 Set");
      Serial.println(incomingString);
      String value= incomingString.substring(10,13);
      Serial.println(value);
      for (uint16_t pulselen = SERVOMIN; pulselen < value.toInt(); pulselen++) {
        pwm.setPWM(servonum, 0, pulselen);
        }
      Serial.println(value);
    }
    // SERVO 3 SETMIN    
    if (incomingString.substring(0,10) == "SERVO0SETR"){
      Serial.println("Servo 0 Set");
      Serial.println(incomingString);
      String value= incomingString.substring(10,13);
      Serial.println(value);
      for (uint16_t pulselen = SERVOMAX; pulselen > value.toInt(); pulselen--) {
        pwm.setPWM(servonum, 0, pulselen);
        }
      Serial.println(value);
    }
    
    // LED 0 Control
    if (incomingString == "LED0OFF"){
      Serial.println("Turning LED0 OFF");
      pwm.setPin(5,0);
    }
    if (incomingString == "LED0ON"){
      Serial.println("Turning LED0 On");
      pwm.setPin(5,4095);
    }

    // LED 1 Control
    if (incomingString == "LED1OFF"){
      Serial.println("Turning LED1 OFF");
      pwm.setPin(6,0);
    }
    if (incomingString == "LED1ON"){
      Serial.println("Turning LED1 On");
      pwm.setPin(6,4095);
    }
      
    // LED 2 Control
    if (incomingString == "LED2OFF"){
      Serial.println("Turning LED2 OFF");
      pwm.setPin(7,0);
    }
    if (incomingString == "LED2ON"){
      Serial.println("Turning LED2 On");
      pwm.setPin(7,4095);
    }

    // LED 3 Control
    if (incomingString == "LED3OFF"){
      Serial.println("Turning LED3 OFF");
      pwm.setPin(8,0);
    }
    if (incomingString == "LED3ON"){
      Serial.println("Turning LED3 On");
      pwm.setPin(8,4095);
    }
      
    // LED 4 Control
    if (incomingString == "LED4OFF"){
      Serial.println("Turning LED4 OFF");
      pwm.setPin(9,0);
    }
    if (incomingString == "LED4ON"){
      Serial.println("Turning LED4 On");
      pwm.setPin(9,4095);
    }
      
    // LED 5 Control
    if (incomingString == "LED5OFF"){
      Serial.println("Turning LED5 OFF");
      pwm.setPin(10,0);
    }
    if (incomingString == "LED5ON"){
      Serial.println("Turning LED5 On");
      pwm.setPin(10,4095);
    }
    
    // LED 6 Control
    if (incomingString == "LED6OFF"){
      Serial.println("Turning LED6 OFF");
      pwm.setPin(11,0);
    }
    if (incomingString == "LED6ON"){
      Serial.println("Turning LED6 On");
      pwm.setPin(11,4095);
    }
    
    // LED 7 Control
    if (incomingString == "LED7OFF"){
      Serial.println("Turning LED7 OFF");
      pwm.setPin(12,0);
    }
    if (incomingString == "LED7ON"){
      Serial.println("Turning LED7 On");
      pwm.setPin(12,4095);
    }
////////////////////////////////////////////////////////////
//////////////////END; COMMANDS TO 9655 ////////////////////
////////////////////////////////////////////////////////////
  Serial.println(incomingString);
  }

}

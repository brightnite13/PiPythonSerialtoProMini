import RPi.GPIO as GPIO
import time
import serial
import Ultrasonics as rangeSensor
GPIO.setmode(GPIO.BCM)

#-----------------GLOBALS---------------------
#Sensor Pins
TRIG1 = 31 
ECHO1 = 33
TRIG2 = 35 
ECHO2 = 37
TRIG3 = 32 
ECHO3 = 26
TRIG4 = 23 
ECHO4 = 24

#------------------PIN SETUP------------------
GPIO.setup(TRIG1,GPIO.OUT)
GPIO.setup(ECHO1,GPIO.IN)
GPIO.setup(TRIG2,GPIO.OUT)
GPIO.setup(ECHO2,GPIO.IN)
GPIO.setup(TRIG3,GPIO.OUT)
GPIO.setup(ECHO3,GPIO.IN)
GPIO.setup(TRIG4,GPIO.OUT)
GPIO.setup(ECHO4,GPIO.IN)

#-------------SERIAL SETUP TO ARDUINO-----------------
ser = serial.Serial(‘/dev/ttyAMA0’, 115200, timeout=1)
    ser.open()

#------------------MAIN------------------------
#(does not execute if imported by another module)
if __name__ == "__main__":
    commandString = input("Enter a Command: ")
        for x in range(0,len(commandString)):
            if commandString[x] == "M":
                ser.write(commandString[x:5]
            if commandString[x] == "S":
                ser.write(commandString[x:5]
            if commandString[x] == "L":
                ser.write(commandString[x:3]
            if commandString[x] == "D":
                ser.write(commandString[x:3]
            if commandString[x] == "W":
                ser.write(commandString[x:4]
            if commandString[x] == "R":
                return rangeSensor.sensorDistance(commandString[x+1])
            

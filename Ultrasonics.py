import RPi.GPIO as GPIO
import time
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

#Pin arrays in case for easy for loop
trigarray = [TRIG1, TRIG2, TRIG3, TRIG4]
echoarray = [ECHO1, ECHO2, ECHO3, ECHO4]

#------------------PIN SETUP------------------
#Just in case running standalone
GPIO.setup(TRIG1,GPIO.OUT)
GPIO.setup(ECHO1,GPIO.IN)
GPIO.setup(TRIG2,GPIO.OUT)
GPIO.setup(ECHO2,GPIO.IN)
GPIO.setup(TRIG3,GPIO.OUT)
GPIO.setup(ECHO3,GPIO.IN)
GPIO.setup(TRIG4,GPIO.OUT)
GPIO.setup(ECHO4,GPIO.IN)

#--------------FUNCTIONS----------------------
def sensorDistance(sensornum)
    GPIO.output(trigarray[sensornum], False)
    
    #wait for sensor to settle
    time.sleep(1)

    #send pulse
    GPIO.output(trigarray[sensornum], True)
    time.sleep(0.00001)
    GPIO.output(trigarray[sensornum], False)

    #wait for trig to read echo
    while GPIO.input(echoarray[sensornum])==0:
        pulse_start = time.time()
    while GPIO.input(echoarray[sensornum])==1:
        pulse_end = time.time()

    #calculate pulse return time in seconds
    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150
    distance = round(distance, 2)

    print "Distance:",distance,"cm"
    return distanceCM

#------------------MAIN------------------------
#(does not execute if imported by another module)
if __name__ == "__main__":
    for x in range(0,4):
        sensorDistance(x)
    GPIO.cleanup()

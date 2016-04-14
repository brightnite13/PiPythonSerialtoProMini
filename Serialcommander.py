import time
import serial

#-------------SERIAL SETUP TO ARDUINO-----------------
ser = serial.Serial(‘/dev/ttyAMA0’, 115200, timeout=1)

#---------------------FUNCTIONS-----------------------
def writeArduino(data):
    ser.open()
    ser.write(data)

def readArduino:
    out = ''
    #wait 1 sec for arduno to respond
    time.sleep(1)
    #if input buffer populated append set & append
    while ser.inWaiting() > 0:
        out += ser.read(1)
    if out != '':
        return out

#------------------MAIN------------------------
#(does not execute if imported by another module)
if __name__ == "__main__":
    commandString = input("Enter a Command: ")
    writeArduino(commandString)
    readArduino()
    ser.close()
    

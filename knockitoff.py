"""
    Notebook for streaming data from a microphone in realtime

    audio is captured using pyaudio
    then converted from binary data to ints using struct
    then displayed using matplotlib

    scipy.fftpack computes the FFT

    if you don't have pyaudio, then run

    >>> pip install pyaudio

    note: with 2048 samples per chunk, I'm getting 20FPS
    when also running the spectrum, its about 15FPS
"""
from PCF8574 import PCF8574_GPIO
from Adafruit_LCD1602 import Adafruit_CharLCD


#import matplotlib.pyplot as plt
import numpy as np
import pyaudio
#from pyqtgraph.Qt import QtGui, QtCore
#import pyqtgraph as pg
import struct
from numpy import average, log10
import math
import sys
import time
import RPi.GPIO as GPIO
import time
from collections import deque
import os
import paramiko


from dotenv import load_dotenv
load_dotenv()

pw = os.environ.get("ROUTER_PW")

# Defines the data bit that is transmitted preferentially in the shiftOut function.
LSBFIRST = 1
MSBFIRST = 2
# define the pins for 74HC595
dataPin   = 11      # DS Pin of 74HC595(Pin14)
latchPin  = 13      # ST_CP Pin of 74HC595(Pin12)
clockPin = 15       # CH_CP Pin of 74HC595(Pin11)
greenPin = 16
redPin = 18
inTimeout = False
timeoutStart = time.time()
print("timeoutStart" + str(timeoutStart))
def rms( data ):
    count = len(data)/2
    format = "%dh"%(count)
    shorts = struct.unpack( format, data )
    sum_squares = 0.0
    for sample in shorts:
        n = sample * (1.0/32768)
        sum_squares += n*n
    return math.sqrt( sum_squares / count )
    
def decibel(rms):
    decibel = 20 * log10(rms)
    return decibel

internetOff = "/usr/sbin/iptables -I FORWARD -s 192.168.1.18 -j DROP && /usr/sbin/iptables -I FORWARD -s 192.168.1.27 -j DROP"
internetOn = "/usr/sbin/iptables -D FORWARD -s 192.168.1.18 -j DROP && /usr/sbin/iptables -D FORWARD -s 192.168.1.27 -j DROP"
#internetOff = "ls -la"
# shiftOut function, use bit serial transmission. 
def shiftOut(dPin,cPin,order,val):
    for i in range(0,8):
        GPIO.output(cPin,GPIO.LOW);
        if(order == LSBFIRST):
            GPIO.output(dPin,(0x01&(val>>i)==0x01) and GPIO.HIGH or GPIO.LOW)
        elif(order == MSBFIRST):
            GPIO.output(dPin,(0x80&(val<<i)==0x80) and GPIO.HIGH or GPIO.LOW)
        GPIO.output(cPin,GPIO.HIGH);

def setup():
    GPIO.setmode(GPIO.BOARD)    # use PHYSICAL GPIO Numbering
    GPIO.setup(dataPin, GPIO.OUT) # set pin to OUTPUT mode
    GPIO.setup(latchPin, GPIO.OUT)
    GPIO.setup(clockPin, GPIO.OUT)
    GPIO.setup(redPin, GPIO.OUT)
    GPIO.setup(greenPin, GPIO.OUT)
    GPIO.output(greenPin, GPIO.HIGH)
    mcp.output(3,1)     # turn on LCD backlight
    lcd.begin(16,2) 

    
# stream constants
CHUNK = 1024 * 2
FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 48000
pause = False

# stream object
p = pyaudio.PyAudio()
stream = p.open(
    input_device_index = 0,
    format=FORMAT,
    channels=CHANNELS,
    rate=RATE,
    input=True,
    output=True,
    frames_per_buffer=CHUNK,
)

def turnInternet(on):
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect("192.168.1.1", username="root", password=pw)
    if(on):
        ssh_stdin, ssh_stdout, ssh_stderr = ssh.exec_command(internetOn)
    else:
        ssh_stdin, ssh_stdout, ssh_stderr = ssh.exec_command(internetOff)
        print("turned internet off")
        print(ssh_stdout)
        print(ssh_stderr)
        
def scale(val, src, dst):
    """
    Scale the given value from the scale of src to the scale of dst.
    """
    return ((val - src[0]) / (src[1]-src[0])) * (dst[1]-dst[0]) + dst[0]

def loop():

    mad = 0
    global inTimeout, timeoutStart
    q = deque(maxlen = 30)
    state = "";
    while True:
        lastState = state
        data = stream.read(CHUNK, exception_on_overflow = False)
        dec = decibel(rms(data))
        whichBar = max(min(int(round(scale(dec, [-70, -30],[8,0]))),8),0)
        q.appendleft(8 - whichBar)
        av = round(average(q)*2)

        if(average(q) > 5):
            GPIO.output(redPin, GPIO.HIGH)
            mad = mad + 1
            state = "DANGER"
            if(mad > 50):
                # GO TO FAIL STATE
                if(not inTimeout):
                    state = "INTERNET OFF"
                    GPIO.output(greenPin, GPIO.LOW)
                    timeoutStart = time.time()
                    inTimeout = True
                    print("turning off")
                    turnInternet(False)
                    print("turned off")
                else:
                    state = "INTERNET OFF"
        else:
            if(mad <= 0):
                state = "OK"
                GPIO.output(redPin, GPIO.LOW)
            mad = max(0, mad - .25)
            if(inTimeout):
                secondsTil = 30 - int(round(time.time() - timeoutStart))
                if(secondsTil % 2 == 1):
                    GPIO.output(redPin, GPIO.HIGH)
                else:
                    GPIO.output(redPin, GPIO.LOW)
                print(secondsTil)
            if(mad == 0 and inTimeout and time.time() - timeoutStart > 30):
                state = "OK"
                print("turning on")
                turnInternet(True)
                print("turned on")
                GPIO.output(greenPin, GPIO.HIGH)
                inTimeout = False
                print("restored")
        GPIO.output(latchPin,GPIO.LOW)  # Output low level to latchPin
        shiftOut(dataPin,clockPin,LSBFIRST,0xFF << whichBar) # Send serial data to 74HC595
        GPIO.output(latchPin,GPIO.HIGH)
        if(state != lastState):
            print("states not equal")
            lcd.clear()
            lcd.setCursor(0,0)
            lcd.message(state)
            
        

def destroy():
    GPIO.output(latchPin,GPIO.LOW)  # Output low level to latchPin
    shiftOut(dataPin,clockPin,LSBFIRST,0x00) # Send serial data to 74HC595
    GPIO.output(latchPin,GPIO.HIGH)
    lcd.clear()
    mcp.output(3,0)
    stream.stop_stream()
    stream.close()
    p.terminate()
    GPIO.cleanup()

    if(inTimeout):
        print("should restore internet")
        turnInternet(True)
    print("should turn off lights")




PCF8574_address = 0x27  # I2C address of the PCF8574 chip.
PCF8574A_address = 0x3F  # I2C address of the PCF8574A chip.
# Create PCF8574 GPIO adapter.
try:
    mcp = PCF8574_GPIO(PCF8574_address)
except Exception as inst:
    print("tried first")
    print(inst)
    try:
        mcp = PCF8574_GPIO(PCF8574A_address)
    except:
        print ('I2C Address Error !')
        exit(1)
# Create LCD, passing in MCP GPIO adapter.
lcd = Adafruit_CharLCD(pin_rs=0, pin_e=2, pins_db=[4,5,6,7], GPIO=mcp)


if __name__ == '__main__':  # Program entrance
    print ('Program is starting...' )
    setup() 
try:
    loop()  
except KeyboardInterrupt:   # Press ctrl-c to end the program.
    destroy()  

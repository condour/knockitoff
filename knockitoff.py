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
import matplotlib.pyplot as plt
import numpy as np
import pyaudio
from pyqtgraph.Qt import QtGui, QtCore
import pyqtgraph as pg
import struct
from numpy import average, log10
import math
import sys
import time
import RPi.GPIO as GPIO
import time

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

def scale(val, src, dst):
    """
    Scale the given value from the scale of src to the scale of dst.
    """
    return ((val - src[0]) / (src[1]-src[0])) * (dst[1]-dst[0]) + dst[0]

def loop():
    while True:
        data = stream.read(CHUNK, exception_on_overflow = False)
        #data_np = struct.unpack(str(2 * CHUNK) + 'B', data)
        dec = decibel(rms(data))
        whichBar = int(round(scale(dec, [-70, -30],[1,7])))
        print(whichBar)
        # if(dec > -35):
            # print("BOOM")
        # elif(dec > -50):
            # print("boom")
        # else:
            # print("...")
        

def destroy():
    stream.stop_stream()
    stream.close()
    p.terminate()
    GPIO.cleanup()

try:
    loop()  
except KeyboardInterrupt:   # Press ctrl-c to end the program.
    destroy()  

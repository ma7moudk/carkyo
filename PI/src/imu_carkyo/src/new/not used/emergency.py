#!/usr/bin/env python

import serial
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BOARD)
import time
port = "/dev/ttyUSB0"
ftdi = serial.Serial(port,9600)
ftdi.flushInput()
inData=''
emerg_pin=16
GPIO.setup(16,GPIO.OUT)

while True:
    GPIO.setup(16,GPIO.OUT)
    time.sleep(1)
    print " low"
    GPIO.setup(16,GPIO.IN)
    time.sleep(1)    
    if (ftdi.inWaiting()>0):
        inData = ftdi.read(ftdi.inWaiting())
        inData = ftdi.readline()
        inData = ftdi.readline().strip()
        print(inData)
    if inData == 'e':
        GPIO.output(emerg_pin , 0 )
        print "emergency warning ... "
    elif inData == 's':
        GPIO.output(emerg_pin , 1 )   
        print " we are OK"     
        

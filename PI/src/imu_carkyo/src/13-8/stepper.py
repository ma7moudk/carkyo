#!/usr/bin/env python

import math, time,rospy
import RPi.GPIO as GPIO
import mmap, os

GPIO.setmode(GPIO.BOARD)
enable = 1
direction = 19
EN=8
PULSE=10 
GPIO.setup(PULSE, GPIO.OUT)
GPIO.setup(EN, GPIO.OUT)
GPIO.setup(direction, GPIO.OUT)
steering_angle = 0.0
started=1
angle_val = 0.0
steering_angle = 0.0
old_angle_val=0.0
last_val = 0.0
new_c = 0.0
new_fb = 0.0
errors=0
errorscmd=0

time.sleep(10)
f_fb = mmap.mmap(os.open('/home/pi/catkin_workspace/src/imu_carkyo/files/angle_fb',os.O_RDWR),1)
f_cmd = mmap.mmap(os.open('/home/pi/catkin_workspace/src/imu_carkyo/files/angle_cmd',os.O_RDWR),1)

while 1:
    t=time.time()
    f_fb.resize(f_fb.size())
    reading = f_fb[:]
    while (len(reading) < 12):
        reading = reading + '2'
    try:    
        new_fb   = float(reading)
        last_val = new_fb
    except ValueError:
        new_fb = last_val
        errors = errors+1
    print ("Size is ------- : ",f_cmd.size())
    if (f_cmd.size()):
        f_cmd.resize(f_cmd.size())
        readingCMD = f_cmd[:]
#    while (len(readingCMD) < 12):
#        reading = readingCMD + '2'
    try:    
        new_c = float(readingCMD)
        last_val = new_c
    except ValueError:
        new_c = last_val
        errorscmd=errorscmd+1
        print "error in receiving cmd , No of errors till now: " , errorscmd
        
    x= time.time()-t  

    if x>0.02:
        tt = 0
        print "Reading 2 files took:",x ," secs"


 #######################################

    if steering_angle != new_c:
        steering_angle = new_c
    if angle_val != new_fb:
        angle_val = new_fb

    steering_angle_deg = float(steering_angle)*180.0/math.pi
#    print "req ang: ",steering_angle_deg , "curr ang" , angle_val
    if (steering_angle_deg > 18.0):
        steering_angle_deg = 18.0
    elif (steering_angle_deg < -18.0):
        steering_angle_deg = -18.0
    print "cmd   ,    fb  " , steering_angle_deg , angle_val
    angle = steering_angle_deg - angle_val
    DIR = ""
    if angle < 0:
        enable = 1
        GPIO.output(EN,0)
        time.sleep(5/1000000.0)
        GPIO.output(direction,1)
        DIR = "RIGHT"
    elif angle > 0:
        enable = 1
        GPIO.output(EN,0)
        time.sleep(5/1000000.0)
        GPIO.output(direction,0)
        DIR = "LEFT"
    else:
        enable = 0

    if started:
        old_angle_val=angle_val
        started=0

 #   if new_fb > 950 or new_fb < 150:
 #       rospy.set_param('/errorCode', 1)
 #       enable=0
                         
    if enable == 1 and abs(angle) >= 0.25 :#was 0.75 #and abs(angle_val-old_angle_val)>3.0:
#        print "some sdteps"
        for x in range(1): #min(50,steps)):
            GPIO.output(PULSE,1)
            time.sleep(3/1000.0)
            GPIO.output(PULSE,0)
            time.sleep(3/1000.0)
        GPIO.output(EN,1)
#        publish_angel_measure_request(1)

#    if(abs(angle_val-old_angle_val)<3.0):
 #       enable=0
    old_angle_val=angle_val

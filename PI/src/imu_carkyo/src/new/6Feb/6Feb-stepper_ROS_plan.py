#!/usr/bin/env python
import rospy , math , time ,datetime
from ackermann_msgs.msg import AckermannDriveStamped
import RPi.GPIO as GPIO
from std_msgs.msg import Int8
import ctypes , mmap , struct , os

GPIO.setmode(GPIO.BOARD)
angle_val = 0.0	#received angle from pot node
enable = 1		# enable stepper change only when angle_val is between -30 , +30  *0.26  0.26 -14 14 

direction = 12
GPIO.setup(10, GPIO.OUT)
GPIO.setup(8, GPIO.OUT)
GPIO.setup(direction, GPIO.OUT)
print "starting"
steering_angle = 0.0
enable_angle_feedback = 0
wait_for_feedback = False
seq = 0
wait_for_callback=True
initialize = True
time.sleep(10)
fd = os.open('/tmp/mmaptesting', os.O_RDONLY)
buf = mmap.mmap(fd,mmap.PAGESIZE, mmap.MAP_SHARED, mmap.PROT_READ)

angle_val = 0.0
steering_angle = 0.0

while 1:
    global steering_angle, angle_val
    new_i, = struct.unpack('f', buf[:4])
    new_s, = struct.unpack('f', buf[4:8])
    if steering_angle != new_i:
        steering_angle = new_i
    if angle_val != new_s:
        angle_val = new_s

    steering_angle_deg = float(steering_angle)*180.0/math.pi
    print "req ang: ",steering_angle_deg , "curr ang" , angle_val
    if (steering_angle_deg > 14.0):
        steering_angle_deg = 14.0
    elif (steering_angle_deg < -14.0):
        steering_angle_deg = -14.0
    angle = steering_angle_deg - angle_val
    DIR = ""
    if angle < 0:
        enable = 1
        GPIO.output(8,0)
        GPIO.output(direction,1)
        DIR = "RIGHT"
    elif angle > 0:
        enable = 1
        GPIO.output(8,0)
        GPIO.output(direction,0)
        DIR = "LEFT"
    else:
        enable = 0

    if enable == 1 and abs(angle) >= 0.75:
        for x in range(5): #min(50,steps)):
            GPIO.output(10,1)
            time.sleep(3/1000.0)
            GPIO.output(10,0)
            time.sleep(3/1000.0)
        GPIO.output(8,1)
#        publish_angel_measure_request(1)

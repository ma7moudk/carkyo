#!/usr/bin/env python

# KZOH: Changed TEN_MICRO to HUNDRED_MICRO (and variables)
# KZOH: Changed function names Thread to: TimmedTasks, EncoderHundredMicroTask, etc.
# KZOH: Removed thread and add a call to TimmedTasks at main
# Add the following functions for Stepper:
#	- ResetStepperPos()
#	- stepper_change(i)

# Accepts motion commands and send it to HW
# Receives HW status: Wheel odom from HW and car controller
##ma7moudk23Oct : seprate mode pins out of switches pins .. Switches value depends on DAC value
#####$$$$$##### Includes #####$$$$$##### 
#####$$$$$##### Includes #####$$$$$##### 
import roslib  
import rospy
import string
import math
import numpy as np
import re
import tf
import time 
from nav_msgs.msg import Odometry

import time 
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BOARD)
###############################################################
###Global values for encoders######
NumberOfInterrputsPorts = 2   # 4 I/Os
HUNDRED_MICRO = 0.0001 # temp = 1 sec (10.0 / 1000.0) / (1000.0)        # units in sec
HunderedMicros = 0
Millies = 0
Secs = 0
Minutes = 0
TimingError = 0
CorrectTiming = 0
WaitCounter = 0

ticksPerSpin = 52
ticksPerSpinMotor = 1024 #936 #1000
diameter = 0.436
lengthBetweenTwoWheels = 1.01
DistancePerCount = (math.pi * diameter) / ticksPerSpin
DistancePerCountMotor = (math.pi * diameter) / ticksPerSpinMotor

Prev = [0,0,0,0]
consecutivezeros = [0,0,0,0]
consecutiveones = [0,0,0,0]
threshold = [2,2,1,1]
Falling = [0,0,0,0]
Rising = [0,0,0,0]
Transition = [0,0,0,0]


pin=[7,11,15,13]  # left - right - speed #[13,15,7,11]
GPIO.setup(pin,GPIO.IN)
#GPIO.setup(pin,GPIO.IN,pull_up_down=GPIO.PUD_UP)

OneMilliesCounter = 0

# 7 m/s = ticksPerSpin*
#for (math.pi * diameter)= 1.369m  >>> 52 ticks
#for                        7   m  >>> 52*(7/1.369)
# 7 m means 52*(7/1.369) ticks
# 7 m in a second mean >> 7/20 m in 1/20 second
# Max possible ticks for 7 m in 1 second  = 7 /  DistancePerCount
# Max possible ticks between 2 time stamps(1/20 s) =  7 / (20* DistancePerCount)  = 13.29

encoderfrequency=20
max_speed=7.0
maxPossibleTicks=max_speed / (encoderfrequency* DistancePerCount)  
# If the sensor can measure speed up to 7 m/s , it can have 13 ticks between 2 time stamps
#( so time of one tick = 1/(20*13) =0.00384 ->  3.8 ms

OldTime = 0.0
OldTicksL = 0.0
OldTicksR = 0.0
OldTicks1 = 0.0
OldTicks2 = 0.0

#angle_pub = rospy.Publisher('/odom/angle', Odometry, queue_size=10)
rospy.init_node('encoder_node', anonymous=True)
odom_pub = rospy.Publisher('/speed_values', Odometry, queue_size=10)
odomMsg = Odometry()


def HundredMilliesTask():
    global Falling ,OldTime, DistancePerCount, DistancePerCountMotor, OldTicksL, OldTicksR, OldTicks1, OldTicks2
    TicksL = Falling [0]
    TicksR = Falling [1]
    Ticks1 = Falling[2]
    Ticks2 = Falling[3]
    CurrentTime = time.time()
    DiffTime = (CurrentTime - OldTime)

    VL = (TicksL-OldTicksL) * DistancePerCount / DiffTime
    VR = (TicksR-OldTicksR) * DistancePerCount / DiffTime
    V1 = (Ticks1-OldTicks1) * DistancePerCountMotor / DiffTime
    V2 = (Ticks2-OldTicks2) * DistancePerCountMotor / DiffTime
 
    #publish Odometry message
    odomMsg.pose.pose.position.x = V1
    odomMsg.pose.pose.position.y = V2
    odomMsg.pose.pose.position.z = 0.0
    odomMsg.twist.twist.linear.x = VL
    odomMsg.twist.twist.linear.y = VR
    odomMsg.twist.twist.linear.z = 0.0  
    odomMsg.twist.twist.angular.x = 0.0
    odomMsg.twist.twist.angular.y = 0.0
    odomMsg.twist.twist.angular.z = 0.0  
    odomMsg.pose.pose.orientation.x = Ticks1 #L
    odomMsg.pose.pose.orientation.y = Ticks2 #R
    odomMsg.pose.pose.orientation.z = 0.0
    odomMsg.header.frame_id = 'odom'
    odomMsg.child_frame_id = 'base_footprint'
    odomMsg.header.stamp= rospy.Time.now()
    odom_pub.publish(odomMsg)


    OldTicksL = TicksL
    OldTicksR = TicksR
    OldTicks1 = Ticks1
    OldTicks2 = Ticks2
    OldTime = CurrentTime       # Ususally it should be 100 millies

def GetRising():
    global Rising
    return (Rising[0], Rising[1],Rising[2],Rising[3])

def GetFalling():
    global Falling
    return (Falling[0], Falling[1],Falling[2],Falling[3])

def GetTransitions():
    global Transitions
    return (Transition[0], Transition[1],Transition[2],Transition[3])

def HundredMicroTask():
    global HunderedMicros, Millies, Secs, Minutes
    HunderedMicros += 1
    if HunderedMicros >=10:
        HunderedMicros=0
        Millies += 1
#        OneMilliesTasks()
        if Millies % 100 == 0.0:
            HundredMilliesTask()
        if Millies >=1000:
            Millies=0
            Secs += 1
            if Secs >=60:	
                Secs=0
                Minutes += 1

def EncoderHundredMicroTask():
    global NumberOfInterrputsPorts, Prev, consecutivezeros, consecutiveones, threshold, Falling, Rising, Transition ,pin
    for i in range(NumberOfInterrputsPorts):
        if GPIO.input(pin[i])==0:
            consecutivezeros[i]+=1
            consecutiveones[i]=0
            if Prev[i] == 1 and consecutivezeros[i] >= threshold[i]:
                Prev[i] = 0
                Falling[i]+=1
                Transition[i]+=1
        else:
            consecutivezeros[i]=0
            consecutiveones[i]+=1
            if Prev[i] == 0 and consecutiveones[i] >= threshold[i]:
                Prev[i] = 1
                Rising[i]+=1
                Transition[i]+=1

def TimmedTasks():
    global TimingError, CorrectTiming, WaitCounter, TimeElapsed
    Start = time.time() #timeit.timeit()
    Next = Start
    Next = Next + HUNDRED_MICRO

    print "Timmes task start"

    #Port[1-n].tmpcount=0, .Rising=0, .Falling=0, .Transition=0, .Prev=0
    while True:
        Now = time.time() # timeit.timeit()
        Start = Now
        if (Now >= Next):	# Exceeds the time
            TimingError+=1
        else:
            CorrectTiming+=1
        while (Now < Next):
            WaitCounter +=1
            Now = time.time() #timeit.timeit()
        EncoderHundredMicroTask()
        HundredMicroTask()
        Next = Next + HUNDRED_MICRO
        End = time.time()
        TimeElapsed = End - Start

if __name__ == '__main__':
    TimmedTasks()

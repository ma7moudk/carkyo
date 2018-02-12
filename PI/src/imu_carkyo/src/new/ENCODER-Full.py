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
NumberOfInterrputsPorts = 4   # 4 I/Os
HUNDRED_MICRO = 0.0001 # temp = 1 sec (10.0 / 1000.0) / (1000.0)        # units in sec
HunderedMicros = 0
Millies = 0
Secs = 0
Minutes = 0
TimingError = 0
CorrectTiming = 0
WaitCounter = 0

ticksPerSpin = 52
ticksPerSpinMotor = 936*1.145 #1000
diameter = 0.436
lengthBetweenTwoWheels = 1.01
DistancePerCount = (math.pi * diameter) / ticksPerSpin   #0.0263410462  #0.0247702498
DistancePerCountMotor = (math.pi * diameter) / ticksPerSpinMotor

Prev = [0,0,0,0]
consecutivezeros = [0,0,0,0]
consecutiveones = [0,0,0,0]
threshold = [6,6,1,1]
Falling = [0,0,0,0]
Rising = [0,0,0,0]
Transition = [0,0,0,0]


pin=[13,15,7,11]
GPIO.setup(pin,GPIO.IN)
#GPIO.setup(pin,GPIO.IN,pull_up_down=GPIO.PUD_UP)

OneMilliesCounter = 0


OldTime = 0.0
OldTicksL = 0.0
OldTicksR = 0.0
OldTicks1 = 0.0
OldTicks2 = 0.0
HalfSecond = rospy.Duration (0.5)

#angle_pub = rospy.Publisher('/odom/angle', Odometry, queue_size=10)
rospy.init_node('encoder_node', anonymous=True)
odom_pub = rospy.Publisher('/speed_values', Odometry, queue_size=10)
odomMsg = Odometry()


def TimmedTasks():
    global DistancePerCount, DistancePerCountMotor, OldTicksL, OldTicksR, OldTicks1, OldTicks2
    global NumberOfInterrputsPorts, Prev, consecutivezeros, consecutiveones, threshold, Falling, Rising, Transition ,pin
    global HalfSecond
    print "Timmes task start"

    #Port[1-n].tmpcount=0, .Rising=0, .Falling=0, .Transition=0, .Prev=0
    NextTime = rospy.Time.now()
    NextTime = NextTime + HalfSecond
    while True:
        for i in range(NumberOfInterrputsPorts):
            if GPIO.input(pin[i]) == 0 :  #potential bug in the original file. It was: if GPIO.input(pin[i])
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

        Now = rospy.Time.now()
        if Now > NextTime:   # ????
            TicksL = Falling[0]
            TicksR = Falling[1]
#            Ticks1 = Falling[2]
#            Ticks2 = Falling[3]           
            DiffTime = (Now - NextTime+HalfSecond)
            print DiffTime ,DiffTime.to_sec(), Now.to_sec() ,NextTime.to_sec()
        #    print "TicksL , TicksR , Ticks1 , Ticks2" ,TicksL , TicksR , Ticks1 , Ticks2
            VL = (TicksL-OldTicksL) * DistancePerCount / DiffTime.to_sec()
            VR = (TicksR-OldTicksR) * DistancePerCount / DiffTime.to_sec()
#            V1 = (Ticks1-OldTicks1) * DistancePerCountMotor / DiffTime.to_sec()
#            V2 = (Ticks2-OldTicks2) * DistancePerCountMotor / DiffTime.to_sec()
        #    print "VL , VR , V1 , V2" ,VL , VR , V1 , V2, min(V1,V2)
            #publish Odometry message
            odomMsg.pose.pose.position.x = 0.0 # V1
            odomMsg.pose.pose.position.y = 0.0 # V2
            odomMsg.pose.pose.position.z = 0.0
            odomMsg.twist.twist.linear.x = VL
            odomMsg.twist.twist.linear.y = VR
            odomMsg.twist.twist.linear.z = 0.0  
            odomMsg.twist.twist.angular.x = 0.0
            odomMsg.twist.twist.angular.y = 0.0
            odomMsg.twist.twist.angular.z = 0.0  
            odomMsg.pose.pose.orientation.x = TicksL
            odomMsg.pose.pose.orientation.y = TicksR
            odomMsg.pose.pose.orientation.z = 0.0
            odomMsg.header.frame_id = 'odom'
            odomMsg.child_frame_id = 'base_footprint'
            odomMsg.header.stamp= rospy.Time.now()
            odom_pub.publish(odomMsg)
            NextTime = NextTime + HalfSecond
            OldTicksL = TicksL
            OldTicksR = TicksR
#            OldTicks1 = Ticks1
#            OldTicks2 = Ticks2
            print "Publishing: " , TicksL, TicksR, VL, VR
            if min(TicksL, TicksR) > 2**15:
                print "restting ticks: " , TicksL, TicksR, OldTicksL, OldTicksR, Falling[0], Falling[1]
                TicksL -= 2**15
                TicksR -= 2**15            
                OldTicksL -= 2**15
                OldTicksR -= 2**15
                Falling[0] -= 2**15
                Falling[1] -= 2**15
                print "restting ticks: " , TicksL, TicksR, OldTicksL, OldTicksR, Falling[0], Falling[1]
if __name__ == '__main__':
    TimmedTasks()

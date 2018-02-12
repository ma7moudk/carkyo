#!/usr/bin/env python
import roslib  
import rospy
import serial
import string
import math
import re
from time import time, sleep
from ackermann_msgs.msg import AckermannDriveStamped

steering_angle=0
speed=0
old_steering_angle=10.0
old_speed=10.0
angle='\x70'
acc='\x50'
enter='\x13'
#flag= false
mode='\x20'
#port = rospy.get_param('device', '/dev/ttyUSB1')
#ser = serial.Serial(port=port, baudrate=9600, timeout=1)
rospy.sleep(5) 

def callback(data):
    global steering_angle
    global speed
    global old_steering_angle
    global old_speed 
    global mode
    global angle
    global acc
    global enter
    global flag
    global ticks
########################1st#################################
    steering_angle=data.drive.steering_angle
    speed=data.drive.speed
    #rospy.loginfo("speed is %f",speed)


########################speed#################################
    if (speed == 0.0) and (speed != old_speed) :
       rospy.loginfo("Brakes on ")
       mode='\x10'
 #      ser.write(mode)
       rospy.loginfo(mode)
         
    elif (speed !=0.0) and (speed != old_speed):
      #rospy.loginfo("Brakes off")
      if speed < 0.0 and (speed != old_speed) :
         #rospy.loginfo("baaaaaaack ")
         if (mode !='\x40'):
           rospy.loginfo("mode wasn't backward ")
           rospy.sleep(1)
           mode='\x40'
  #         ser.write(mode)
         rospy.sleep(1)
         #acc='\x5f'
         speedTemp=speed
         speedTemp=translate(speedTemp, -0.20, 0.00, 95, 80)
         rospy.loginfo(speedTemp)
         #acc=hex(int(speedTemp))
         acc=chr(speedTemp)


      elif speed > 0.0 and (speed != old_speed):
         #rospy.loginfo("forwaaard ")
         if (mode !='\x30'):
           rospy.loginfo("mode wasn't forward ")
           rospy.sleep(1)
           mode='\x30'
   #       ser.write(mode)
           rospy.loginfo(mode)
         rospy.sleep(1)
         #acc='\x5f'
         speedTemp=speed
         speedTemp=translate(speedTemp, 0.00, 0.20, 80, 95)
         rospy.loginfo(speedTemp)
         #acc=hex(int(speedTemp))
         acc=chr(speedTemp)


    
    # ser.write(acc)
      rospy.loginfo(acc)

########################speed#################################
    if (steering_angle == 0.0) and (steering_angle != old_steering_angle) :
       rospy.loginfo("steer off")
       mode='\x70'
 #      ser.write(mode)
       rospy.loginfo(mode)
         
    elif (steering_angle !=0.0) and (steering_angle != old_steering_angle):
      if steering_angle < 0.0 and (steering_angle != old_steering_angle) :
         if (mode !='\x80'):
           rospy.loginfo("mode wasn't negative angle ")
           rospy.sleep(1)
           mode='\x80'
  #        ser.write(mode)
           rospy.loginfo(mode)
           rospy.sleep(1)



      elif steering_angle > 0.0 and (steering_angle != old_steering_angle):
         if (mode !='\x90'):
           rospy.loginfo("mode wasn't positive angle ")
           rospy.sleep(1)
           mode='\x90'
   #       ser.write(mode)
           rospy.loginfo(mode)
           rospy.sleep(1)
         

#############################################################################3
    old_steering_angle=steering_angle
    old_speed=speed


def encoder():
    line = ser.readline()
    words = string.split(line,",")
    rospy.loginfo(line)
    
    
def listener():
    rospy.loginfo(10)
    rospy.init_node('ser_pub', anonymous=True)
    rospy.Subscriber("/rbcar_robot_control/command", AckermannDriveStamped, callback)
    rospy.spin()


def translate(value, leftMin, leftMax, rightMin, rightMax):
    # Figure out how 'wide' each range is
    leftSpan = leftMax - leftMin
    rightSpan = rightMax - rightMin

    # Convert the left range into a 0-1 range (float)
    valueScaled = float(value - leftMin) / float(leftSpan)

    # Convert the 0-1 range into a value in the right range.
    return rightMin + (valueScaled * rightSpan)

if __name__ == '__main__':
    listener()
    encoder()


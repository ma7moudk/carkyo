#!/usr/bin/env python
# Accepts motion commands and send it to HW
# Receives HW status: Wheel odom from HW and car controller

#####$$$$$##### Includes #####$$$$$##### 
import roslib  
import rospy
import serial
import string
import math
import numpy as np
import re
from time import time, sleep
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance
from geometry_msgs.msg import TwistWithCovariance
from std_msgs.msg import Header
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import TransformStamped

#####$$$$$##### Global variables #####$$$$$##### 
tickR= 0.0
tickL= 0.0
direction = 1
x=0.0
y=0.0
th =0.0
i=1
j=0
t=0.0
xx=10
yy=12
tickRight=[0,0,0,0,0,0,0,0,0,0]
tickLeft=[0,0,0,0,0,0,0,0,0,0]
current=[0,0,0,0,0,0,0,0,0,0]
last_counter=0
current_counter=0
starting=1
ckeck =0
PotInitialized = False
potentiometer1 = 0
potentiometer2 = 0
potentiometer3 = 0
potentiometer4 = 0
PotIndex = 1
last_time=0.0
ticksPerSpin = 52
diameter = 0.41
lengthBetweenTwoWheels = 1.01
DistancePerCount = (math.pi * diameter) / ticksPerSpin
DistancePer9Count =DistancePerCount * 9

steering_angle=0
speed=0
old_steering_angle=0.0
old_speed=10.0
angle='\x70'
acc='\x50'
enter='\x13'
#flag= false
mode='\x20'

#####$$$$$##### ROS init #####$$$$$##### 
odom_pub = rospy.Publisher('/wheel/odom', Odometry, queue_size=10)
angle_pub = rospy.Publisher('/odom/angle', Odometry, queue_size=10)
odomMsg = Odometry()
angleMsg = Odometry()
odomQuat = Quaternion()

odomMsg.pose.covariance = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

odomMsg.twist.covariance = [0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1]


default_port='/dev/ttyUSB2'
port = rospy.get_param('device', default_port)
ser = serial.Serial(port=port, baudrate=9600,  timeout=1)
rospy.sleep(2) 

#####$$$$$##### keyboard callback #####$$$$$##### 
def callback(data):
    global steering_angle,speed,old_steering_angle,old_speed ,mode,angle,acc,enter,flag
    print("#############################################")
    encoder()
    #get_speed()

########################1st#################################

    steering_angle=data.drive.steering_angle
    speed=data.drive.speed
########################speed#################################
    if (speed == 0.0) and (speed != old_speed) :
       rospy.loginfo("normal on ")
       mode='\x20'
       ser.write(mode)
       rospy.loginfo(mode)
         
    elif (speed !=0.0) :
      if speed == -0.45 and (speed != old_speed) :
         if (mode !='\x40'):
           rospy.loginfo("mode wasn't backward ")
           rospy.sleep(1)
           mode='\x40'
           ser.write(mode)
           rospy.sleep(3)
###########
      if speed == 0.45 and (speed != old_speed) :
         if (mode !='\x30'):
           rospy.loginfo("mode wasn't forward ")
           rospy.sleep(1)
           mode='\x30'
           ser.write(mode)
           rospy.sleep(2)
      elif speed < -0.45 and (speed != old_speed) :
         #rospy.loginfo("baaaaaaack ")
         if (mode !='\x40'):
           rospy.loginfo("mode wasn't backward ")
           rospy.sleep(1)
           mode='\x40'
           ser.write(mode)
           rospy.sleep(2)
         #acc='\x5f'
         #test_acc=95
         speedTemp=speed
         speedTemp=translate(speedTemp, -5.00, -0.90, 95, 85)
         speedTemp2=int(speedTemp)
         rospy.loginfo(speedTemp2)
         #acc=hex(int(speedTemp))
         acc=chr(speedTemp2)
         #acc=chr(int(test_acc))

      elif speed > 0.45 and (speed != old_speed):
         #rospy.loginfo("forwaaard ")
         if (mode !='\x30'):
           rospy.loginfo("mode wasn't forward ")
           rospy.sleep(1)
           mode='\x30'
           ser.write(mode)
           rospy.loginfo(mode)
           rospy.sleep(5)
         speedTemp=speed
         speedTemp=translate(speedTemp, 0.90, 5.00, 85, 95) 
  #55 - 5f >> 0.00 0.20 then 0.04 >> 57
         speedTemp2=int(speedTemp)
         rospy.loginfo(speedTemp2)
         #acc=hex(int(speedTemp))
         acc=chr(speedTemp2)
         #acc=chr(int(test_acc))
    
      ser.write(acc)
      rospy.loginfo(acc)
########################steering#################################
   # if (steering_angle == 0.0) and (steering_angle != old_steering_angle) :
    #   rospy.loginfo("steer off")
      # mode='\x70'
     #  ser.write('\x70')
      # rospy.loginfo('\x70')
         
    if (steering_angle != old_steering_angle):
      if steering_angle < 0.0 and (steering_angle < old_steering_angle) :
         steeringTemp=steering_angle
         steeringTemp=translate(steeringTemp, -0.70, 0.00, 159, 145)
         steeringTemp2=int(steeringTemp)
         rospy.loginfo("1st condition more negative")
         rospy.loginfo(steeringTemp2)
         angle=chr(steeringTemp2)
         ser.write(angle)
         #rospy.sleep(1)

      elif steering_angle < 0.0 and (steering_angle > old_steering_angle) :
         steeringTemp=steering_angle
         steeringTemp=translate(steeringTemp, -0.70, 0.00, 129, 132)
         steeringTemp2=int(steeringTemp)
         rospy.loginfo(steeringTemp2)
         rospy.loginfo("2nd condition less negative")
         angle=chr(steeringTemp2)
         ser.write(angle)
         #rospy.sleep(1)

      elif steering_angle >= 0.0 and (steering_angle > old_steering_angle):
         steeringTemp=steering_angle
         steeringTemp=translate(steeringTemp, 0.00, 0.70, 129, 143)
         steeringTemp2=int(steeringTemp)
         rospy.loginfo(steeringTemp2)
         rospy.loginfo("3rd condition more positive ")
         angle=chr(steeringTemp2)
         ser.write(angle)
         #rospy.sleep(1)

      elif steering_angle >= 0.0 and (steering_angle < old_steering_angle):
         steeringTemp=steering_angle
         steeringTemp=translate(steeringTemp, 0.00, 0.70, 149, 145)
         steeringTemp2=int(steeringTemp)
         rospy.loginfo("4th condition less positive ")
         rospy.loginfo(steeringTemp2)
         angle=chr(steeringTemp2)
         ser.write(angle)
         #rospy.sleep(1)
 
         
    old_steering_angle=steering_angle
    old_speed=speed

####################################################################3


def encoder():
    global previousL , previousR ,last_time ,current_time , x ,y ,th ,ticksPerSpin , diameter ,lengthBetweenTwoWheels, DistancePer9Count ,DistancePerCount, direction, PotInitialized, PotIndex, potentiometer1,potentiometer2, potentiometer3, potentiometer4 , minV , vx
    
    global i,tickRight , tickLeft ,current ,last_counter , current_counter , starting

    checksum = 0
    num_of_words = 12 + checksum
    bytes_of_one_line=67 + 3*checksum
   # rospy.sleep(0.25)
    no_of_bytes=ser.inWaiting()

    print "no_of_bytes" ,no_of_bytes
      
    if no_of_bytes > bytes_of_one_line * 2:
      data=ser.read(no_of_bytes)
      all_lines=string.split(data,"\n") 
      no_of_lines =len(all_lines)
      line=all_lines[no_of_lines-2]
      print line

#    check=0
#    for count in range (0,68):
#      check = check + ord(line[count])
      line = line.replace(":",",")
      words = string.split(line,",")
      ####words = [0,1,2,3,4,5]

  #    if  int(words[12],16)!=check+14 :
  #      print("..........Waaaaaaaaaaarning ... Serial error")

      if len(words) == num_of_words and words[0]=="Ver":# and  int(words[12],16)==check+14 :
        #print("line is correct")
        #rospy.loginfo(int(words[12],16))
        tickR = float(int(words[5],16))
        tickL = float(int(words[6],16))
        if PotIndex == 1:
            potentiometer1 = float(int(words[11],16))
        elif PotIndex == 2:
            potentiometer2 = float(int(words[11],16))
        elif PotIndex == 3:
            potentiometer3 = float(int(words[11],16))
        else:
            potentiometer4 = float(int(words[11],16))
        #print(float(int(words[11],16)))
        PotIndex=PotIndex+1
        if PotIndex > 4:
            PotInitialized = True
            PotIndex = 1
        current_time = float(int(words[2],16))/1000.0
        if starting==1 :
          for s in range (0,10):
            tickRight[s]=tickR
            tickLeft[s]=tickL
            current[s]=current_time
          starting=0

        if PotInitialized == True:
            pot = (potentiometer1 + potentiometer2 + potentiometer3 +potentiometer4) / 4
            if pot <= 1600.0 :
              fBetaRads= 0.663225166 
            elif pot > 1600.0 and pot <= 1750.0 :
              fBetaRads= 0.5235987756 
            elif pot > 1750.0 and pot <= 1980.0 :
              fBetaRads= 0.34906585 
            elif pot > 1980.0 and pot <= 2170.0 :
              fBetaRads= 0.1745329252 
            elif pot > 2170.0 and pot <= 2303.0 :
              fBetaRads= 0.0 
            elif pot > 2303.0 and pot <= 2420.0 :
              fBetaRads= -0.1745329252
            elif pot > 2420.0 and pot <= 2530.0 :
              fBetaRads= -0.34906585
            elif pot > 2530.0  :
              fBetaRads= -0.4188790205

            angleMsg.pose.pose.position.x=fBetaRads
            angleMsg.header.frame_id = 'odom'
            angleMsg.child_frame_id = 'base_link'
            angleMsg.header.stamp= rospy.Time.now()
            #print("angle")
            #print(angleMsg.pose.pose.position.x)
  #####################################################
        tickRight[i]=tickR
        tickLeft[i]=tickL
        current[i]=current_time
        i=i+1
        if i>9 :
          i=0

        current_counter=i-1
        last_counter=i
        if current_counter < 0:
          current_counter=9
        if last_counter > 9:
          last_counter=0
        #print current
        #print tickLeft
        #print tickRight

  #########################################################
        counter_before_current=current_counter-1
        if (counter_before_current < 0):
          counter_before_current = 9
  #########################################################


  ####################option1#####################################
        deltaL = tickLeft[current_counter] - tickLeft[counter_before_current]
        deltaR = tickRight[current_counter] - tickRight[counter_before_current]
        dt = current[current_counter] - current[counter_before_current]
        #rospy.loginfo("time[%d]=%f \n time[%d]=%f",current_counter, current[current_counter] ,counter_before_current ,current[counter_before_current] )


        if "R" in words[3]:
          direction = 0
          print("direction is RRRRRRRRRRRRRRRRR ")
          vL= -(deltaL * DistancePerCount)/dt
          vR= -(deltaR * DistancePerCount)/dt
        elif "F" in words[3]:
          direction = 1
          print("direction is FFFFFFFFFFFFFFFFF ")
          vL= (deltaL * DistancePerCount)/dt
          vR= (deltaR * DistancePerCount)/dt

        else:
          direction = 2
          #print("Paaaarking ")
          #vL= 0 #commented only for debugging should be 0
          #vR= 0
          vL= (deltaL * DistancePerCount)/dt
          vR= (deltaR * DistancePerCount)/dt


        vx= (vR + vL)/ 2.0
        vy= 0.0

        vth=(vR - vL)/ lengthBetweenTwoWheels
        delta_x = (vx * math.cos(th)) * dt
        delta_y = (vx * math.sin(th)) * dt
        delta_th = vth * dt
        x += delta_x
        y += delta_y
        th += delta_th
  ################### ma7moudk ### add this tonormalize th ###
        while (th >= math.pi):
          th -= 2.0 * math.pi
        while (th <= (-math.pi)):
          th += 2.0 * math.pi
  ############################################################


        odomMsg.twist.twist.linear.x = vx
        odomMsg.twist.twist.linear.y = vy
        odomMsg.twist.twist.linear.z = 0.0
   
        odomMsg.twist.twist.angular.x = 0.0
        odomMsg.twist.twist.angular.y = 0.0
        odomMsg.twist.twist.angular.z = vth

        odomMsg.header.frame_id = 'odom'
        odomMsg.child_frame_id = 'base_link'
        odomMsg.header.stamp= rospy.Time.now()
        #print("position")
        #print(odomMsg.pose.pose.position)
        #print("linearvx,angularvz")
        #print(odomMsg.twist.twist.linear.x)
        #print(odomMsg.twist.twist.angular.z)

    #p= rospy.Time.now()
        odom_pub.publish(odomMsg)


def listener():
    rospy.loginfo(10)
    rospy.init_node('ser_pub', anonymous=True)
    rospy.Subscriber("/rbcar_robot_control/command", AckermannDriveStamped, callback,queue_size=1)
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
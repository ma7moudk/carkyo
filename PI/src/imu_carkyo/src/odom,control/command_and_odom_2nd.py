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
import time 
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance
from geometry_msgs.msg import TwistWithCovariance
from std_msgs.msg import Header
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import TransformStamped

#####$$$$$##### Global variables #####$$$$$##### 
allign=0
vL=0.0
vR=0.0
dt=0.0
mode_time=time.time()
tickR= 0.0 
tickL= 0.0
x=0.0
y=0.0
th =0.0
i=1 
t=0.0
tickRight=[0,0,0,0,0,0,0,0,0,0]
tickLeft=[0,0,0,0,0,0,0,0,0,0]
current=[0,0,0,0,0,0,0,0,0,0]
last_counter=0
current_counter=0
starting=1
ckeck =0
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
mode='\x20'
speedTemp2=0.0
#####$$$$$##### Global variables for(speedometer) #####$$$$$##### 
minV=0.0
previousTicks=0.0
previousTicks2=0.0
deltaTicks=0.0
deltaTicks2=0.0
ticks=0.0
ticks2=0.0
previousTime=0.0
currentTime=0.0005
Vmps=0.0
Vmps2=0.0
vx=0.0
ticksPerSpin_sensor = 1000*2.9/2.3
DistancePerCount_sensor = (math.pi * diameter) / ticksPerSpin_sensor



#####$$$$$##### ROS init #####$$$$$##### 
odom_pub = rospy.Publisher('/wheel_odometry', Odometry, queue_size=10)
angle_pub = rospy.Publisher('/odom/angle', Odometry, queue_size=10)
odomMsg = Odometry()
angleMsg = Odometry()

odomMsg.pose.covariance = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

odomMsg.twist.covariance = [0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1]

#####$$$$$##### Serial init #####$$$$$##### 
default_port='/dev/ttyUSB1'
default_port_ard='/dev/ttyUSB1'
from serial.tools import list_ports
#ports=list_ports.comports()
#for p in ports:
#    if "ttyUSB" in p[0]:
#        default_port =  p[0]
#    if "ttyACM in p[0]:
#        default_port_ard =  p[0]
port = rospy.get_param('device', default_port)
ser = serial.Serial(port=port, baudrate=9600,  timeout=1)
#port_ard = rospy.get_param('device', default_port_ard)
#ser_ard = serial.Serial(port=port_ard,baudrate=115200, timeout=1)

rospy.sleep(0.25)
ser.write('\x10') 
rospy.sleep(0.25)

#####$$$$$##### keyboard callback #####$$$$$##### 
def callback(data):
    global mode_time,steering_angle,speed,old_steering_angle,old_speed ,mode,angle,acc,enter,speedTemp2
    encoder()
 #   get_speed()
   # print("###########################################")
    print time.time()
    steering_angle=data.drive.steering_angle
    speed=data.drive.speed
    allign=data.drive.steering_angle_velocity
   # print "speed , steering_angle" , speed , steering_angle
	

            #########speed#############
    if (allign == 1.0)  : 
      rospy.loginfo('\x1b[1M\r'
                  '\033[34;1mAlligned: \033[32;1m%0.2f ',
                  allign)
      ser.write('\x00')

    rospy.loginfo('\x1b[1M\r'
                  '\033[34;1mAlligned: \033[32;1m%0.2f ',
                  allign)

    if (speed >= -0.02) and (speed <= 0.02)  : ##### Case 0
		 #rospy.loginfo("normal on ")
		 mode='\x20'
		 ser.write(mode)
		# rospy.loginfo(mode)
		 speedTemp=0.0
		 speedTemp2=int(speedTemp)
		# print "case0 0" ,speedTemp,speedTemp2
		 acc=chr(speedTemp2)

    if (speed != old_speed):
      print "different speed"

      if (speed < -0.02 ) and (speed > -0.90) :  ##### Case 1
         if (mode !='\x40'):
           rospy.loginfo("mode wasn't backward ")
           mode='\x40'
           ser.write(mode)
           mode_time=time.time()
         speedTemp=0.0
         speedTemp2=int(speedTemp)
         print "case1 -0.45" ,speedTemp,speedTemp2
         acc=chr(speedTemp2)


      elif (speed > 0.02 ) and (speed < 0.90) :  ##### Case 2
         if (mode !='\x30'):
           rospy.loginfo("mode wasn't forward ")
           mode='\x30'
           ser.write(mode)
           mode_time=time.time()
         speedTemp=0.0
         speedTemp2=int(speedTemp)
         print "case2 0.45" ,speedTemp,speedTemp2
         acc=chr(speedTemp2)

      elif speed <= -0.905 : ##### Case 3
         speedTemp=speed
         speedTemp=translate(speedTemp, -5.00, -0.90, 95, 88)
         speedTemp2=int(speedTemp)
         print "case3 <= -0.91" ,speedTemp,speedTemp2
         acc=chr(speedTemp2)

      elif speed >= 0.905 : ##### Case 4
         speedTemp=speed
         speedTemp=translate(speedTemp, 0.90, 5.00, 88, 95) 
         speedTemp2=int(speedTemp)
         print "case4 >= 0.91" ,speedTemp,speedTemp2
         acc=chr(speedTemp2)

      else : ##### Case 5
         print "speedTemp2 , mode" , speedTemp2 , mode      

      if(time.time()-mode_time>1.0):
		     ser.write(acc)
		     print "acc" , acc

            #########steering#############
         
    if (steering_angle != old_steering_angle):
      if steering_angle < 0.0 and (steering_angle < old_steering_angle) :
         steeringTemp=steering_angle
         steeringTemp=translate(steeringTemp, -0.70, 0.00, 159, 145)
         steeringTemp2=int(steeringTemp)
         rospy.loginfo("1st condition more negative")
         rospy.loginfo(steeringTemp2)
         angle=chr(steeringTemp2)
         ser.write(angle)


      elif steering_angle < 0.0 and (steering_angle > old_steering_angle) :
         steeringTemp=steering_angle
         steeringTemp=translate(steeringTemp, -0.70, 0.00, 129, 132)
         steeringTemp2=int(steeringTemp)
         rospy.loginfo(steeringTemp2)
         rospy.loginfo("2nd condition less negative")
         angle=chr(steeringTemp2)
         ser.write(angle)


      elif steering_angle >= 0.0 and (steering_angle > old_steering_angle):
         steeringTemp=steering_angle
         steeringTemp=translate(steeringTemp, 0.00, 0.70, 129, 143)
         steeringTemp2=int(steeringTemp)
         rospy.loginfo(steeringTemp2)
         rospy.loginfo("3rd condition more positive ")
         angle=chr(steeringTemp2)
         ser.write(angle)


      elif steering_angle >= 0.0 and (steering_angle < old_steering_angle):
         steeringTemp=steering_angle
         steeringTemp=translate(steeringTemp, 0.00, 0.70, 149, 145)
         steeringTemp2=int(steeringTemp)
         rospy.loginfo("4th condition less positive ")
         rospy.loginfo(steeringTemp2)
         angle=chr(steeringTemp2)
         ser.write(angle)
      else:
         print "steeringTemp2" ,steeringTemp2
 
         
    old_steering_angle=steering_angle
    old_speed=speed

####################################################################3


def encoder():
    global previousL , previousR ,last_time ,current_time , x ,y ,th ,ticksPerSpin , diameter ,lengthBetweenTwoWheels, DistancePer9Count ,DistancePerCount, minV , vx , vL ,vR , dt
    
    global i,tickRight , tickLeft ,current ,last_counter , current_counter , starting

    checksum = 1
    num_of_words = 8
    bytes_of_one_line=44
    no_of_bytes=ser.inWaiting()

   # print "no_of_bytes" ,no_of_bytes
      
    if no_of_bytes > bytes_of_one_line * 2:
      data=ser.read(no_of_bytes)
      all_lines=string.split(data,"\n") 
      no_of_lines =len(all_lines)
      line=all_lines[no_of_lines-2]
      print line

      check=0
      for count in range ( 0 , (bytes_of_one_line - 4 ) ):  # somehow strange why -1 
        check = check + ord(line[count])

      words = string.split(line,",")
      print "received vs sent" , check , int(words[7],16) 

      if  int(words[7],16)!=check :
        print("..........Waaaaaaaaaaarning ... Serial error")
      if len(words) == num_of_words and words[0]=="V" and  int(words[7],16)==check :
        tickR = float(int(words[5],16))
        tickL = float(int(words[6],16))

        current_time = float(int(words[2],16))/1000.0

        if starting==1 :
          for s in range (0,10):
            tickRight[s]=tickR
            tickLeft[s]=tickL
            current[s]=current_time
          starting=0

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

  #########################################################
        counter_before_current=current_counter-1
        if (counter_before_current < 0):
          counter_before_current = 9

        deltaL = tickLeft[current_counter] - tickLeft[counter_before_current]
        deltaR = tickRight[current_counter] - tickRight[counter_before_current]
        dt = current[current_counter] - current[counter_before_current]
	#print "deltaL , deltaR" , deltaL , deltaR
	print words[3]

        if "R" in words[3]:
          print("direction is RRRRRRRRRRRRRRRRR ")
          vL= -(deltaL * DistancePerCount)/dt
          vR= -(deltaR * DistancePerCount)/dt
          a=-1

        elif "F" in words[3]:
          print("direction is FFFFFFFFFFFFFFFFF ")
          vL= (deltaL * DistancePerCount)/dt
          vR= (deltaR * DistancePerCount)/dt
          a=1

        else:
          print("Paaaarking ")
          vL= 0 #commented only for debugging should be 0
          vR= 0
         # vL= (deltaL * DistancePerCount)/dt
        #  vR= (deltaR * DistancePerCount)/dt
          a=0

        #vx= a * minV
        vx= (vR + vL)/ 2.0
	#print "vx", vx
        vy= 0.0
        vth=(vR - vL)/ lengthBetweenTwoWheels
       # delta_x = (vx * math.cos(th)) * dt
       # delta_y = (vx * math.sin(th)) * dt
       # delta_th = vth * dt
       # x += delta_x
       # y += delta_y
       # th += delta_th
  ### add this tonormalize th ###
       # while (th >= math.pi):
       #   th -= 2.0 * math.pi
       # while (th <= (-math.pi)):
       #   th += 2.0 * math.pi

        odomMsg.twist.twist.linear.x = vx
        odomMsg.twist.twist.linear.y = vy
        odomMsg.twist.twist.linear.z = 0.0  
        odomMsg.twist.twist.angular.x = 0.0
        odomMsg.twist.twist.angular.y = 0.0
        odomMsg.twist.twist.angular.z = vth
        odomMsg.header.frame_id = 'odom'
        odomMsg.child_frame_id = 'base_footprint'
        odomMsg.header.stamp= rospy.Time.now()
        odom_pub.publish(odomMsg)
	print "vx , vth" , vx , vth


def get_speed():
    global previousTicks,previousTicks2,deltaTicks,deltaTicks2,vx,ticksPerSpin_sensor,DistancePerCount_sensor,minV,ticks,ticks2,previousTime,currentTime,Vmps,Vmps2


   # line = ser_ard.readline()
####replacing last line with these 7 lines
    bytes_of_one_line_ard=78
    no_of_bytes=ser_ard.inWaiting()
    if no_of_bytes > bytes_of_one_line_ard * 2:
      data_ard=ser_ard.read(no_of_bytes)
      all_lines_ard=string.split(data_ard,"\n") 
      no_of_lines_ard =len(all_lines_ard)
      line=all_lines_ard[no_of_lines_ard-2]


 #   print "get speed line"
      print line
      words_ard = string.split(line,",")
      if len(words_ard) > 7:
      	try:
			    ticks = int(words_ard[1])
			    ticks2 = int(words_ard[3])
			    previousTime =float(words_ard[5])
			    currentTime =float(words_ard[7])
			    deltaTicks = ticks - previousTicks
			    deltaTicks2 = ticks2 - previousTicks2
      	except Exception as e:
	      	print e

      dt=(currentTime - previousTime)*0.001
      if dt == 0 :
   			dt =0.00001
      Vmps= (deltaTicks * DistancePerCount_sensor)/dt
      Vmps2= (deltaTicks2 * DistancePerCount_sensor)/dt
      previousTicks = ticks
      previousTicks2 = ticks2
      minV=min(Vmps,Vmps2) 
    #  print "encoder vs speedometer" , vx , minV

def listener():
    rospy.init_node('wheel_odometry_node', anonymous=True)
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

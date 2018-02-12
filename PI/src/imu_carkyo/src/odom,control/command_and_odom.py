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
import tf
import time 
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance
from geometry_msgs.msg import TwistWithCovariance
from std_msgs.msg import Header
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Imu

#####$$$$$##### Global variables #####$$$$$##### 
yaw=0.0
ang_x=0.0
ang_y=0.0
ang_z=0.0
fBetaRads=0.0
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
current_step=0
current_sign=0
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
vy=0.0
ticksPerSpin_sensor = 1000*2.9/2.3
DistancePerCount_sensor = (math.pi * diameter) / ticksPerSpin_sensor



#####$$$$$##### ROS init #####$$$$$##### 
rospy.init_node('wheel_odometry_node', anonymous=True)
#rospy.Timer(rospy.Duration(0.1), pub_callback, oneshot=False)
odom_pub = rospy.Publisher('/wheel_odometry', Odometry, queue_size=10)
#angle_pub = rospy.Publisher('/odom/angle', Odometry, queue_size=10)
odomMsg = Odometry()
angleMsg = Odometry()

odomMsg.pose.covariance = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

odomMsg.twist.covariance = [0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1]

#####$$$$$##### Serial init #####$$$$$##### 
default_port='/dev/ttyACM0'

from serial.tools import list_ports
#ports=list_ports.comports()
#for p in ports:
#    if "ttyUSB" in p[0]:
#        default_port =  p[0]
port = rospy.get_param('device', default_port)
ser = serial.Serial(port=port, baudrate=9600,  timeout=1)

rospy.sleep(0.25)
ser.write('\x10') 
rospy.sleep(0.25)

#####$$$$$##### keyboard callback #####$$$$$##### 
def callback(data):
    global mode_time,steering_angle,speed,old_steering_angle,old_speed ,mode,angle,acc,enter,speedTemp2 , current_step ,current_sign
    encoder()

    print("###########################################")
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
      ser.write('\x03')

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
     #    if( current_step < 31 and current_sign ==-1):
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


      elif steering_angle > 0.0 and (steering_angle > old_steering_angle):
    #     if( current_step < 31 and current_sign ==1):
	     steeringTemp=steering_angle
	     steeringTemp=translate(steeringTemp, 0.00, 0.70, 129, 143)
	     steeringTemp2=int(steeringTemp)
	     rospy.loginfo(steeringTemp2)
	     rospy.loginfo("3rd condition more positive ")
	     angle=chr(steeringTemp2)
	     ser.write(angle)


      elif steering_angle > 0.0 and (steering_angle < old_steering_angle):
	     steeringTemp=steering_angle
	     steeringTemp=translate(steeringTemp, 0.00, 0.70, 149, 145)
	     steeringTemp2=int(steeringTemp)
	     rospy.loginfo("4th condition less positive ")
	     rospy.loginfo(steeringTemp2)
	     angle=chr(steeringTemp2)
	     ser.write(angle)
 #     else:
  #       print "steeringTemp2" ,  steeringTemp2
 
         
    old_steering_angle=steering_angle
    old_speed=speed

####################################################################3


def encoder():
    global previousL , previousR ,last_time ,current_time , x ,y ,th ,ticksPerSpin , diameter ,lengthBetweenTwoWheels, DistancePer9Count ,DistancePerCount, minV , vx ,vy , vL ,vR , dt
    
    global i,tickRight , tickLeft ,current ,last_counter , current_counter , starting , current_step ,current_sign
    global yaw , ang_x , ang_y , ang_z ,fBetaRads,current_step
    checksum = 1
    num_of_words = 8+2
    bytes_of_one_line=44+5
    no_of_bytes=ser.inWaiting()

    print "no_of_bytes" ,no_of_bytes
      
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
      print "received vs sent" , check , int(words[9],16) 

      if  int(words[9],16)!=check :
        print("..........Waaaaaaaaaaarning ... Serial error")
      if len(words) == num_of_words and words[0]=="V" and  int(words[9],16)==check :
        tickR = float(int(words[5],16))
        tickL = float(int(words[6],16))
        current_step = int(words[8],16)
        step2fBetaRads()
        if "R" in words[7]:
        	current_sign = -1
        if "L" in words[7]:
        	current_sign = 1

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
        #  vL= 0 #commented only for debugging should be 0
         # vR= 0
          vL= (deltaL * DistancePerCount)/dt
          vR= (deltaR * DistancePerCount)/dt
          a=0

        v_mps= (vR + vL)/ 2.0

        vx = v_mps * math.cos(fBetaRads) * math.cos(yaw)  #fBetaRads is trurning angle , robot_pose_pa_ is yaw
        vy = v_mps * math.cos(fBetaRads) * math.sin(yaw)


def pub_callback(event):
    global vx , vy , ang_x , ang_y , ang_z , vL ,vR,fBetaRads
    odomMsg.twist.twist.linear.x = vx
    odomMsg.twist.twist.linear.y = vy
    odomMsg.twist.twist.linear.z = 0.0  
    odomMsg.twist.twist.angular.x = ang_x
    odomMsg.twist.twist.angular.y = ang_y
    odomMsg.twist.twist.angular.z = ang_z  
    odomMsg.pose.pose.orientation.x = vR
    odomMsg.pose.pose.orientation.y = vL
    odomMsg.pose.pose.orientation.z =fBetaRads
    odomMsg.header.frame_id = 'odom'
    odomMsg.child_frame_id = 'base_footprint'
    odomMsg.header.stamp= rospy.Time.now()
    odom_pub.publish(odomMsg)
    print "..... publishing Wheel odometry ...."

rospy.Timer(rospy.Duration(0.1), pub_callback)


def callbackImu(data):
    global yaw , ang_x , ang_y , ang_z
    quat_x=data.orientation.x
    quat_y=data.orientation.y
    quat_z=data.orientation.z
    quat_w=data.orientation.w
    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion([quat_x, quat_y, quat_z, quat_w])
    ang_x=data.angular_velocity.x
    ang_y=data.angular_velocity.y
    ang_z=data.angular_velocity.z
#    yaw=yaw+((math.pi)/2.0)
#    if(yaw>math.pi):
#        yaw-=2*math.pi
        
    print "Callback IMU -- yaw: " , yaw*180.0/3.14



def listener():
    rospy.init_node('wheel_odometry_node', anonymous=True)
    rospy.Subscriber("/rbcar_robot_control/command", AckermannDriveStamped, callback,queue_size=1)
    rospy.Subscriber("/imu_enu",Imu, callbackImu,queue_size=1)
    rospy.spin()

def step2fBetaRads():
    global fBetaRads,current_step ,current_sign
    fBetaRads=current_sign*current_step*2
    print "fBetaRads" , fBetaRads
    fBetaRads=fBetaRads*math.pi/180.0


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

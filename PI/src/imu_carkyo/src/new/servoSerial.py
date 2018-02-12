#!/usr/bin/env python

import serial
import rospy,math
import time ,tf
import numpy as np
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Quaternion
quat=Quaternion()


port = "/dev/ttyACM0"
ard = serial.Serial(port,9600)
ard.flushInput()
rospy.init_node('steering_angle_node')

servo_val=0.0
imuMsg = Imu() 
yaw=0.0
def callback_IMU(data): 
    global servo_val , imuMsg, yaw
    if (ard.inWaiting()>0):
        servo_val = ard.readline().strip()


    orientation_list = [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w] 
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    yaw=yaw*180.0/math.pi
    print yaw , servo_val
    with open ("arduinomod1.csv","a") as text_file:
        text_file.write("{0},{1}\n".format(yaw , servo_val))

    
def listener():
    rospy.Subscriber("/imu_enu_pi", Imu, callback_IMU)
    rospy.spin()
    
  
if __name__ == '__main__':
    listener()

#!/usr/bin/env python

'''
steering_angle.py:
    Determines the steering angle from potentiometer
'''

__author__ = 'George Kouros'
__license__ = 'GPLv3'
__maintainer__ = 'George Kouros'
__email__ = 'gkourosg@yahoo.gr'
import sys,serial
import roslib
import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float64
import sys, select, termios, tty
import thread ,time
from numpy import clip


port = "/dev/ttyACM0"
ard = serial.Serial(port,9600)
ard.flushInput()
rospy.init_node('steering_angle_node')
steering_pub = rospy.Publisher('rbcar_robot_control/steering_angle', AckermannDriveStamped, queue_size=10)
steering_msg = AckermannDriveStamped()

def steering_callback(event):
	if (ard.inWaiting()>0):
		pot_val = ard.readline().strip()
		steering_angle = pot_val
		print(pot_val)
    steering_msg.header.stamp=rospy.Time.now()
    steering_msg.drive.speed = 0.0
    steering_msg.drive.steering_angle = steering_angle
    steering_msg.drive.steering_angle_velocity = 0.0
    steering_pub.publish(steering_msg)
    
if __name__ == '__main__':
    #listener()
    rospy.Timer(rospy.Duration(0.1), steering_callback, oneshot=False)
#    keyop = AckermannDriveStampedKeyop(sys.argv[1:len(sys.argv)])

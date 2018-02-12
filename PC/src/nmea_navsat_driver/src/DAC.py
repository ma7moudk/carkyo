#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
import tf
import math , time

rospy.init_node('DAC_listener', anonymous=True)   
vel=0.0
time_vel=rospy.Time.now()
time_DAC=rospy.Time.now()

x = True

def callback_odom(data): 
    global vel , time_vel
    vel=(data.twist.twist.linear.x + data.twist.twist.linear.y)/2.0
    time_vel=data.header.stamp

def callback_command(data):
    global dac ,time_DAC, x
    dac=data.drive.speed
    time_DAC = data.header.stamp
    rospy.set_param('SPEEDPARAM', dac)
#    with open("ws/opfiles/1.csv", "a") as text_file:
#        text_file.write("{0},{1},{2},{3}\n".format(dac ,vel ,time_DAC ,time_vel))
#        text_file.write("%d,%f,%f,%f \n" %dac %vel %time_DAC %time_vel )

def listener():
    rospy.Subscriber("/speed_values", Odometry, callback_odom)
    rospy.Subscriber("/rbcar_robot_control/command", AckermannDriveStamped, callback_command)
    rospy.spin()

if __name__ == '__main__':
    listener()

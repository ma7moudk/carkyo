#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
import tf
import math , time

rospy.init_node('DAC_listener', anonymous=True)   
xx=0.0
yy=0.0
time_vel=rospy.Time.now()
time_DAC=rospy.Time.now()
dac =0.0

def callback_odom(data): 
    global xx ,yy , time_vel
    xx=data.pose.pose.position.x 
    yy=data.pose.pose.position.y
    time_vel=data.header.stamp


def callback_command(data): 
    global dac ,time_DAC
    dac=data.drive.speed
    time_DAC = data.header.stamp
    print "dsVZF"

    with open("ws/gpsfolder/firstfile.csv", "a") as text_file:
        text_file.write("{0},{1},{2},{3}\n".format(xx ,yy ,dac ,time_vel))
#        text_file.write("%d,%f,%f,%f \n" %dac %vel %time_DAC %time_vel )

def listener():
    rospy.Subscriber("/odometry/gps", Odometry, callback_odom)
    rospy.Subscriber("/pot_angle", AckermannDriveStamped, callback_command)
    rospy.spin()

if __name__ == '__main__':
    listener()

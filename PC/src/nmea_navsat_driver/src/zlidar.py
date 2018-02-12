#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
import math , time
rospy.init_node('zlidar_listener', anonymous=False)   


#tim=rospy.Time.now()

def callback(data):
	print "Lidar 1",data.ranges[5],data.ranges[6],data.ranges[7]


def callback2(data):
	print "Lidar 2",data.ranges[5],data.ranges[6],data.ranges[7]


#rospy.Timer(rospy.Duration(0.1), pub_imu)

def listener():
    rospy.Subscriber("/lidar", LaserScan, callback)
    rospy.Subscriber("/lidar2", LaserScan, callback2)
    rospy.spin()

if __name__ == '__main__':
    listener()

#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
import math , time
rospy.init_node('check_node', anonymous=True)   
odom_pub = rospy.Publisher('/wheel_odometry_new', Odometry, queue_size=10)


def callback(event): 
    odomMsg = Odometry()
    odomMsg.header.stamp= rospy.Time.now()
    odom_pub.publish(odomMsg)


if __name__ == '__main__':
	rospy.Timer(rospy.Duration(0.5), callback)
	rospy.spin()

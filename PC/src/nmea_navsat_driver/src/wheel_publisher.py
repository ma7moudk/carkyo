#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
import math , time
rospy.init_node('listener', anonymous=True)   
odom_pub = rospy.Publisher('/wheel_odometry_new', Odometry, queue_size=10)
odomMsg = Odometry()

def callback_odom(data): 
    odomMsg=data
    odomMsg.header.stamp= rospy.Time.now()
    odom_pub.publish(odomMsg)
    odomMsg.twist.covariance = [0.0001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1000, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1000, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1000, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1000]
    odomMsg.pose.covariance = [0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


def listener():
    rospy.Subscriber("/wheel_odometry", Odometry, callback_odom)
    rospy.spin()

if __name__ == '__main__':
    listener()

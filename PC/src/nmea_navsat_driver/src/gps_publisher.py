#!/usr/bin/env python
import rospy
from sensor_msgs.msg import NavSatFix
import math , time
rospy.init_node('listener', anonymous=True)   
gps_pub = rospy.Publisher('/fix1', NavSatFix, queue_size=10)
gpsMsg = NavSatFix()


def callback_GPS(data): 
	gpsMsg=data
	gpsMsg.header.stamp=rospy.Time.now()
	gps_pub.publish(gpsMsg)
	print gpsMsg


def listener():
    rospy.Subscriber("/fix", NavSatFix, callback_GPS)
    rospy.spin()

if __name__ == '__main__':
    listener()

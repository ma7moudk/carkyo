#!/usr/bin/env python
import rospy
from std_msgs.msg import Int8
import math , time
rospy.init_node('servo_publish', anonymous=False)   
servo_pub = rospy.Publisher('/servo', Int8, queue_size=10)
servoMsg = Int8()

while True:
	for i in range (-30,30,5):
		servoMsg.data = i
		servo_pub.publish(servoMsg)
		time.sleep(0.1)
	for i in range (30,-30,-5):
		servoMsg.data = i
		servo_pub.publish(servoMsg)
		time.sleep(0.1)


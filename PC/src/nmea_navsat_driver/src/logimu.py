#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import Imu
import math , time
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Quaternion
quat=Quaternion()

rospy.init_node('check_imu', anonymous=False)  
imuMsg = Imu() 
imuMsg2 = Imu() 
yaw2 = 0

def callback_IMU(data): 
	global num_y, y_all , quat , imuMsg, yaw2
	imuMsg = data
	orientation_list = [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w] 
	(roll, pitch, yaw) = euler_from_quaternion (orientation_list)
	yaw=yaw*180.0/math.pi
	print "Yaw: ",yaw," , Yaw After: ",yaw2

def callback_IMU2(data): 
	global num_y, y_all , quat , imuMsg2, yaw2
	imuMsg2 = data
	orientation_list = [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w] 
	(roll, pitch, yaw2) = euler_from_quaternion (orientation_list)
	yaw2=yaw2*180.0/math.pi

def listener():
    rospy.Subscriber("/imu_enu", Imu, callback_IMU)
    rospy.Subscriber("/imu_enu_new", Imu, callback_IMU2)
    rospy.spin()



def translate(value, leftMin, leftMax, rightMin, rightMax):
    leftSpan = leftMax - leftMin
    rightSpan = rightMax - rightMin
    valueScaled = float(value - leftMin) / float(leftSpan)
    return rightMin + (valueScaled * rightSpan)

if __name__ == '__main__':
    listener()

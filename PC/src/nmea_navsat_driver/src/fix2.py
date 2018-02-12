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
offset=0.0
angle =0.0
num_y=0.0
y_all = 0.0
rospy.init_node('fix2_imu', anonymous=False)  
imu_pub = rospy.Publisher('/imu_enu_new', Imu, queue_size=2)
imuMsg = Imu() 

def callback_IMU(data): 
	global num_y, y_all , quat , imuMsg
	imuMsg = data
	imuMsg.header.stamp=rospy.Time.now()
	imu_pub.publish(imuMsg)

def callback_IMU1(data): 
	global num_y, y_all , quat , imuMsg
	imuMsg = data
	orientation_list = [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w] 
	(roll, pitch, yaw) = euler_from_quaternion (orientation_list)
	Imu=yaw*180.0/math.pi
	corrected = -1.73399627454e-14 * (Imu**7) + 1.50971951556e-12 * (Imu**6) + 1.7564078874e-09 * (Imu**5) - 9.50916176219e-08 * (Imu**4) - 4.57021085153e-05 * (Imu**3) + 0.00198426448012 * (Imu**2) + 1.22039838734 * (Imu**1) + 1.65794223227
	corrected = corrected + offset  
	if corrected >= 180:
		corrected -= 360
	if corrected <= -180:
		corrected += 360
	with open("new2.csv" , "a" ) as txt:
		txt.write("{0},{1}\n".format(Imu,corrected))
	new_yaw= corrected*math.pi/180.0	
	quat = quaternion_from_euler (roll, pitch, new_yaw)
	imuMsg.orientation.x = quat[0]
	imuMsg.orientation.y = quat[1]
	imuMsg.orientation.z = quat[2]
	imuMsg.orientation.w = quat[3]
	imuMsg.header.stamp=rospy.Time.now()
	imu_pub.publish(imuMsg)

def listener():
    rospy.Subscriber("/imu_enu_pi2", Imu, callback_IMU)
    rospy.spin()

def translate(value, leftMin, leftMax, rightMin, rightMax):
    leftSpan = leftMax - leftMin
    rightSpan = rightMax - rightMin
    valueScaled = float(value - leftMin) / float(leftSpan)
    return rightMin + (valueScaled * rightSpan)

if __name__ == '__main__':
    listener()

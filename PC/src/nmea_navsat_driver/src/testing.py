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

angle =0.0
num_y=0.0
y2 = 0.0
y_all = 0.0

rospy.init_node('enu_listener', anonymous=False)   

def callback_IMU(data): 
    global num_y, y_all , quat, y2
    orientation_list = [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w] 
    (roll, pitch, y) = euler_from_quaternion (orientation_list)
    print "y : ", y*180.0/math.pi, "\t" ,y2*180.0/math.pi , " ",data.angular_velocity.x

def callback_IMU2(data): 
    global y2
    orientation_list = [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w] 
    (roll, pitch, y2) = euler_from_quaternion (orientation_list)

#rospy.Timer(rospy.Duration(0.1), pub_imu)

def listener():
    rospy.Subscriber("/imu_enu_new", Imu, callback_IMU)
    rospy.Subscriber("/imu_enu", Imu, callback_IMU2)
    rospy.spin()

if __name__ == '__main__':
    listener()

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

rospy.init_node('enu_listener', anonymous=False)   
imu_pub = rospy.Publisher('/imu/data1', Imu, queue_size=10)
imuMsg = Imu()

def callback_IMU(data): 
    global angle
    imuMsg = data
    yaw =angle
    quat = quaternion_from_euler (0.0, 0.0, yaw)
    imuMsg.orientation.x = quat[0]
    imuMsg.orientation.y = quat[1]
    imuMsg.orientation.z = quat[2]
    imuMsg.orientation.w = quat[3]
    imuMsg.angular_velocity.x = 0.0
    imuMsg.angular_velocity.y = 0.0
    imuMsg.angular_velocity.z = 0.0
    imuMsg.linear_acceleration.x = 0.0
    imuMsg.linear_acceleration.y = 0.0
    imuMsg.linear_acceleration.z = 9.8  
    imu_pub.publish(imuMsg)
    print imuMsg

def callback_ack(data):
    global angle
    angle = (data.drive.jerk)* math.pi/180.0

#rospy.Timer(rospy.Duration(0.1), pub_imu)

def listener():
    rospy.Subscriber("/imu/data", Imu, callback_IMU)
    rospy.Subscriber("/rbcar_robot_control/command", AckermannDriveStamped, callback_ack)
    rospy.spin()

if __name__ == '__main__':
    listener()

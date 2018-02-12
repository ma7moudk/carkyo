#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import tf
import math , time
rospy.init_node('multi_listener', anonymous=True)   

posx=4.0
posy=0.0
yaw=0.0
def callback_odom(data): 
    global posx,posy
    posx=data.pose.pose.position.x
    posy=data.pose.pose.position.y

def callback_imu(data): 
    global yaw , posx , posy
    x=data.orientation.x
    y=data.orientation.y
    z=data.orientation.z
    w=data.orientation.w
    (a, b, yaw) = tf.transformations.euler_from_quaternion([x, y, z, w])
    print "yaw" , yaw*180.0/math.pi , "x , y " , posx ,",", posy
    
def listener():
    rospy.Subscriber("/odometry/gps", Odometry, callback_odom)
    rospy.Subscriber("/imu_enu", Imu, callback_imu)
    rospy.spin()

if __name__ == '__main__':
    listener()

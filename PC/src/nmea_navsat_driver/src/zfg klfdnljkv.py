#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from ackermann_msgs.msg import AckermannDriveStamped

import math , time
rospy.init_node('enu_listener', anonymous=False)   
imu_pub = rospy.Publisher('/hamada', Imu, queue_size=10)
imuMsg = Imu()

ang_x=0.0
ang_y=0.0
ang_z=0.0
x=0.0
y=0.0
z=0.0
w=0.0
acc_x=0.0
acc_y=0.0
acc_z=0.0
#tim=rospy.Time.now()

def callback_IMU(data): 
    w = data.orientation.w
    print "oju lllll"

def callback(data):
    while 1:
        print rospy.get_param("SPEEDPARAM"),"HAMAFDA"


#rospy.Timer(rospy.Duration(0.1), pub_imu)

def listener():
    rospy.Subscriber("/wheel_odometry", Odometry, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()


#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import math , time
rospy.init_node('enu_listener', anonymous=False)   
imu_pub = rospy.Publisher('/imu_enu_new', Imu, queue_size=10)
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
    global ang_x , ang_y ,ang_z , x, y, z, w,acc_x , acc_y ,acc_z
    w = data.orientation.w
    x = data.orientation.x
    y = data.orientation.y
    z = data.orientation.z
    ang_x =data.angular_velocity.x 
    ang_y =data.angular_velocity.y
    ang_z =data.angular_velocity.z
    acc_x =data.linear_acceleration.x
    acc_y =data.linear_acceleration.y
    acc_z =data.linear_acceleration.z
    imuMsg.angular_velocity.x = ang_x
    imuMsg.angular_velocity.y = ang_y
    imuMsg.angular_velocity.z = ang_z
    imuMsg.linear_acceleration.x = acc_x
    imuMsg.linear_acceleration.y = acc_y
    imuMsg.linear_acceleration.z = acc_z 
    imuMsg.orientation.w = w
    imuMsg.orientation.x = x
    imuMsg.orientation.y = y
    imuMsg.orientation.z = z
    imuMsg.header.frame_id = 'imu_link'
    imuMsg.header.stamp= data.header.stamp # rospy.Time.now()
#    print ("SSSSStime diff" , "")
#    print (rospy.Time.now() , data.header.stamp)
#    print (rospy.Time.now() - data.header.stamp)
#    print ("")
    imu_pub.publish(imuMsg)
    imuMsg.orientation_covariance = [0.00001, 0 , 0, 0 , 0.00001, 0, 0 , 0 , 0.00001]
    imuMsg.angular_velocity_covariance = [0.01, 0 , 0, 0 , 0.01, 0, 0 , 0 , 0.0001]
    imuMsg.linear_acceleration_covariance = [0.01 , 0 , 0, 0 , 0.01, 0, 0 , 0 , 0.01]



#rospy.Timer(rospy.Duration(0.1), pub_imu)

def listener():
    rospy.Subscriber("/imu_enu", Imu, callback_IMU)
    rospy.spin()

if __name__ == '__main__':
    listener()

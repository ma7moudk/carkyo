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
y_all = 0.0
rospy.init_node('fix_imu', anonymous=False)  
imu_pub = rospy.Publisher('/imu_enu_new', Imu, queue_size=10)
imuMsg = Imu() 

def callback_IMU1(data): 
	global num_y, y_all , quat , imuMsg
	imuMsg = data
	imuMsg.header.stamp=rospy.Time.now()
	imu_pub.publish(imuMsg)

def callback_IMU(data): 
	global num_y, y_all , quat , imuMsg
	imuMsg = data
	orientation_list = [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w] 
	(roll, pitch, yaw) = euler_from_quaternion (orientation_list)
	yaw=yaw*180.0/math.pi
	if ( yaw >=	96.5966 and yaw <152.8278 ):
	    new_yaw=translate(yaw, 96.5966, 152.8278, 98.7, 143.5)
	    case = 1
	    
	elif ( yaw >= 152.8278 or yaw < -127.0044 ):
		if yaw<0:
			n_yaw=yaw+360
		else:
			n_yaw=yaw  # 152.82 180 , 180 , (-180 > -152.8 >> 180 >> 127  >> 360 >> 233

		new_yaw=translate(n_yaw, 152.8278, -127.0044+360.0, 143.5, -125.02+360.0)
		if new_yaw>180.0:
			new_yaw=new_yaw-360.0
		case = 2
	    	    	
	elif ( yaw >=	-127.0044 and yaw <-89.3822 ):
	    new_yaw=translate(yaw, -127.0044, -89.3822, -125.02, -81.62)
	    case = 3
	    	
	elif ( yaw >=   -89.3822 and yaw <-49.0744 ):
	    new_yaw=translate(yaw, -89.3822, -49.0744, -81.62, -34.16)
	    case = 4
	    	
	elif ( yaw >= -49.0744 and yaw <-46.3617 ):
	    new_yaw=translate(yaw, -49.0744, -46.3617, -34.16, -38.08)
	    case = 5
	    		
	elif ((yaw >= -46.3617 and yaw <=0) or (yaw <44.6031 and yaw > 0) ):
	    n_yaw=yaw+180.0		
	    new_yaw=translate(n_yaw, -46.3617+180.0, 44.6031+180.0, -38.08+180.0, 54.6+180.0)
	    new_yaw=new_yaw-180.0	    
	    case = 6
	    
	elif ( yaw >= 44.6031 and yaw <96.5966 ):
	    new_yaw=translate(yaw, 44.6031, 96.5966, 54.6, 98.7)
	    case = 7
	    	
	else:
	    print "there is something wrong , yaw val : ", yaw
	    case =  8

#	print case, "yaw" , yaw , "new_yaw" , new_yaw
	new_yaw= new_yaw*math.pi/180.0		
	quat = quaternion_from_euler (roll, pitch, new_yaw)
	imuMsg.orientation.x = quat[0]
	imuMsg.orientation.y = quat[1]
	imuMsg.orientation.z = quat[2]
	imuMsg.orientation.w = quat[3]
	#imuMsg.angular_velocity.x = case
	imuMsg.header.stamp=rospy.Time.now()
	imu_pub.publish(imuMsg)

#rospy.Timer(rospy.Duration(0.1), pub_imu)

def listener():
    rospy.Subscriber("/imu_enu", Imu, callback_IMU)
    rospy.spin()



def translate(value, leftMin, leftMax, rightMin, rightMax):
    leftSpan = leftMax - leftMin
    rightSpan = rightMax - rightMin
    valueScaled = float(value - leftMin) / float(leftSpan)
    return rightMin + (valueScaled * rightSpan)

if __name__ == '__main__':
    listener()

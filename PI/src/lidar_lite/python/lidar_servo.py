#!/usr/bin/env python
import rospy,math
from sensor_msgs.msg import LaserScan
import math , time
from lidar_lite import Lidar_Lite
import RPi.GPIO as GPIO
import numpy as np
from std_msgs.msg import Int8

GPIO.setmode(GPIO.BOARD)
GPIO.setup(37, GPIO.OUT)
servo = GPIO.PWM(37, 50)
servo.start(1.0/18.0*(90)+2)
time.sleep(0.25)


rospy.init_node('lidar2_node', anonymous=True)
lidar_msg=LaserScan()
lidar_pub= rospy.Publisher('lidar2', LaserScan, queue_size=2)

lidar = Lidar_Lite()
connected = lidar.connect(1)
dist_local  = [30.0]*14
dist_global = [30.0]*14


def callback_servo(data):
    global dist_local,dist_global
    t=time.time()

    if connected < -1:
        print "Not Connected"
    
    lidar_msg.range_min = 0.01 # 1 cm
    lidar_msg.range_max = 30.0  # 6 m    
    servo.ChangeDutyCycle(1.0/18.0*(88-data.data)+2)



#    dist_local[(data.data+30)/5] = distance
    if data.data == 30:
        for i in range(13,-1,-1):
            distance = lidar.getDistance()/100.0
            if distance < 0.1 :
                distance = 30.0
            dist_local[i] = distance
            
    elif data.data == -10:
        for i in range(0,14):
            distance = lidar.getDistance()/100.0
            if distance < 0.1 :
                distance = 30.0
            dist_local[i] = distance
            

    dist_global = dist_local

    lidar_msg.angle_min = -30.0*math.pi/180.0
    lidar_msg.angle_max = 10.0*math.pi/180.0
    lidar_msg.angle_increment = 2.86*math.pi/180.0 # angle resolution is 1 deg    
    # if laserscan give 1 scan per second so time of full scan is 1 sec , time of one beam =1/140.0=0.00714285s
    lidar_msg.time_increment = 0.022
    lidar_msg.scan_time = 0.31
    lidar_msg.ranges = dist_global
    lidar_msg.header.frame_id='hokuyo1_link'
    lidar_msg.header.stamp=rospy.Time.now()
    lidar_pub.publish(lidar_msg)  
    print time.time()-t      
    print ("publishing lidar2")
        
        
def listener():
    rospy.Subscriber("/servo",Int8,callback_servo)


if __name__ == '__main__':
    listener()
    rospy.spin()
    

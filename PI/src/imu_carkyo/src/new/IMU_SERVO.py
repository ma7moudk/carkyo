#!/usr/bin/env python
import rospy ,math
import time ,tf
import numpy as np
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import RPi.GPIO as GPIO
from std_srvs.srv import Empty
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Quaternion
quat=Quaternion()
GPIO.setmode(GPIO.BOARD)

GPIO.setup(7,GPIO.OUT)
servo = GPIO.PWM(7, 50)
servo.start(1.0/18.0*(0)+2)
time.sleep(0.5)
time.sleep(0.5)
servo.start(1.0/18.0*(0)+2)
time.sleep(0.25)
rospy.init_node('ImuServo_node', anonymous=True)
data=Imu()
angle = 0
inc = 1
yaw = 0.0

while 1:

    for i in range(0,181,4):
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(7,GPIO.OUT)
        servo = GPIO.PWM(7, 50)
        servo.start(1.0/18.0*(i)+2)
        time.sleep(0.2)
        GPIO.cleanup()
        for j in range(0,51):
            data=rospy.wait_for_message("/imu_enu_pi", Imu )
            orientation_list = [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w] 
            (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
            yaw=yaw*180.0/math.pi

            print "Anlge: ",i," , Yaw: ",yaw
            with open("angle.csv", "a") as text_file:
                text_file.write("{0},{1}\n".format(i ,yaw))


    servo.ChangeDutyCycle(1.0/18.0*(0)+2)
    time.sleep(0.5)
    


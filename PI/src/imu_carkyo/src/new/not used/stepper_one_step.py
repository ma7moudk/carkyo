#!/usr/bin/env python
import rospy ,math
import time 
from ackermann_msgs.msg import AckermannDriveStamped
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BOARD)
stepper_pos=0
old_steering_angle=0.0

rospy.init_node('steering_one_step', anonymous=True)
angle_pub = rospy.Publisher('/angle_one_step', AckermannDriveStamped, queue_size=10)
angleMsg = AckermannDriveStamped()
direction = 12
GPIO.setup(10,GPIO.OUT)
GPIO.setup(8,GPIO.OUT)
GPIO.setup(direction,GPIO.OUT)

def callback_relative(data):
    global stepper_pos ,old_steering_angle ,angle_pub , angleMsg
    steering_angle=data.drive.steering_angle
    allign=data.drive.steering_angle_velocity
    print "stepper_pos" , stepper_pos
	
    if (allign == 1.0)  : 
        rospy.loginfo('\x1b[1M\r'
                    '\033[34;1mAlligned: \033[32;1m%0.2f ',
                    allign)
        stepper_pos =0
    elif (allign == 0.0)  : 
        rospy.loginfo('\x1b[1M\r'
                  '\033[34;1mAlligned: \033[32;1m%0.2f ',
                  allign)
         
    if (steering_angle != old_steering_angle):
        if steering_angle < -0.001 and (steering_angle < old_steering_angle) :
            stepper_change_relative(-1)

        elif steering_angle < -0.001 and (steering_angle > old_steering_angle) :
            stepper_change_relative(1)

        elif steering_angle > 0.001 and (steering_angle > old_steering_angle):
            stepper_change_relative(1)

        elif steering_angle > 0.001 and (steering_angle < old_steering_angle):
            stepper_change_relative(-1)
            
        else:
            print ("zero value do nothing" )        
                  
    old_steering_angle=steering_angle
    angleMsg.drive.steering_angle=stepper_pos
    angle_pub.publish(angleMsg)

def stepper_change_relative(i):
    global stepper_pos , direction
    print "Num of steps : ",i
    stepper_pos = stepper_pos + i
    if i < 0:
        GPIO.output(8,0)
        GPIO.output(direction,1)
        i = i*-1
    elif i > 0:
        GPIO.output(8,0)
        GPIO.output(direction,0)

    for x in range(0,i*20):
        GPIO.output(10,1)
        time.sleep(2.0/1000.0)
        GPIO.output(10,0)
        time.sleep(2.0/1000.0)
    GPIO.output(8,1)
    

def listener():
    rospy.Subscriber("/rbcar_robot_control/command2", AckermannDriveStamped, callback_relative,queue_size=1)
    rospy.spin()

if __name__ == '__main__':
    listener()

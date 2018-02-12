#!/usr/bin/env python
import rospy ,math
import time 
from ackermann_msgs.msg import AckermannDriveStamped
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BOARD)
stepper_pos=0
old_steering_angle=0.0

rospy.init_node('steering_node', anonymous=True)
angle_pub = rospy.Publisher('/angle_value', AckermannDriveStamped, queue_size=10)
angleMsg = AckermannDriveStamped()
direction = 12
GPIO.setup(10,GPIO.OUT)
GPIO.setup(8,GPIO.OUT)
GPIO.setup(direction,GPIO.OUT)

def callback(data):
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
        if steering_angle < 0.0 and (steering_angle < old_steering_angle) :
            steeringTemp=steering_angle
            steeringTemp=translate(steeringTemp, -0.70, 0.04, -8, -1)
            rospy.loginfo("1st condition more negative")
            stepper_change(int(steeringTemp))

        elif steering_angle < 0.0 and (steering_angle > old_steering_angle) :
            steeringTemp=steering_angle
            steeringTemp=translate(steeringTemp, -0.70, 0.04, 6, 1)
            rospy.loginfo("2nd condition less negative")
            stepper_change(int(steeringTemp))

        elif steering_angle > 0.0 and (steering_angle > old_steering_angle):
            steeringTemp=steering_angle
            steeringTemp=translate(steeringTemp, 0.04, 0.70,1, 8)
            rospy.loginfo("3rd condition more positive")
            stepper_change(int(steeringTemp))

        elif steering_angle > 0.0 and (steering_angle < old_steering_angle):
            steeringTemp=steering_angle
            steeringTemp=translate(steeringTemp, 0.04, 0.70, -1, -6)
            rospy.loginfo("4th condition less positive ")
            stepper_change(int(steeringTemp))
          
    old_steering_angle=steering_angle
    angleMsg.drive.steering_angle=stepper_pos
    angle_pub.publish(angleMsg)

def stepper_change(i):
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
    rospy.Subscriber("/rbcar_robot_control/command1", AckermannDriveStamped, callback,queue_size=1)
    rospy.spin()

def translate(value, leftMin, leftMax, rightMin, rightMax):
    # Figure out how 'wide' each range is
    leftSpan = leftMax - leftMin
    rightSpan = rightMax - rightMin
    # Convert the left range into a 0-1 range (float)
    valueScaled = float(value - leftMin) / float(leftSpan)
    # Convert the 0-1 range into a value in the right range.
    return rightMin + (valueScaled * rightSpan)

if __name__ == '__main__':
    listener()

#!/usr/bin/env python

'''
steering_angle.py:
    Determines the steering angle from potentiometer
'''
import serial
import rospy
from ackermann_msgs.msg import AckermannDriveStamped


port = "/dev/ttyACM0"
ard = serial.Serial(port,9600)
ard.flushInput()
rospy.init_node('steering_angle_node')
steering_pub = rospy.Publisher('pot_angle', AckermannDriveStamped, queue_size=10)
steering_msg = AckermannDriveStamped()
steering_angle = 0.0
brakesEn = 0
print_val=0.0

def steering_callback(event):
    global steering_angle, brakesEn , print_val
    lastVal = 0.0
    pot_val = ""
    if (brakesEn == 1):
        ard.write('e')
    else:
        ard.write('s')
        
    if (ard.inWaiting()>0):
        pot_val = ard.read(ard.inWaiting())
        pot_val = ard.readline()
        pot_val = ard.readline().strip()
        print(pot_val)
    
    try:
        lastVal = steering_angle
        steering_angle = float(pot_val)
        print_val=steering_angle
        steering_angle = (-0.04*steering_angle)+20
    except ValueError:
        steering_angle=lastVal
	print ("error")
    print (steering_angle+2.63)
    steering_msg.header.stamp=rospy.Time.now()
    steering_msg.drive.speed = print_val
    steering_msg.drive.steering_angle = steering_angle+2.59
    steering_msg.drive.steering_angle_velocity = 0.0
    steering_pub.publish(steering_msg)

def callback (data):
    global brakesEn
    if (data.drive.speed == 0):
        brakesEn = 1
        print "speed is zero applying brakes , time now is " , rospy.Time.now()
    else:
        brakesEn = 0
    
def listener():
    rospy.Subscriber("/rbcar_robot_control/command", AckermannDriveStamped, callback,queue_size=1)
    rospy.spin()    
    
  
if __name__ == '__main__':
    rospy.Timer(rospy.Duration(0.1), steering_callback, oneshot=False)
    listener()

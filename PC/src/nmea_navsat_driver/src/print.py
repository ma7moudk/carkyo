#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
import math , time

rospy.init_node('print', anonymous=True)   

pot_angle=0.0
command_angle=0.0
command_speed=0.0
sensor_speed=0.0

def sensor_speed(data):
    global sensor_speed
    sensor_speed=(data.twist.twist.linear.x+data.twist.twist.linear.y)/2.0


def command(data):
    global command_angle,command_speed 
    command_angle=data.drive.steering_angle
    command_speed=data.drive.speed
    print "sens_speed,",sensor_speed,",com_speed,",command_speed,",pot_angle,",pot_angle,",com_angle,",command_angle,",time,",rospy.Time.now()


def pot_angle(data):
    global pot_angle
    pot_angle=data.drive.steering_angle
    
def listener():
    rospy.Subscriber("/speed_values", Odometry, sensor_speed,queue_size=1)
    rospy.Subscriber("/rbcar_robot_control/command", AckermannDriveStamped, command,queue_size=1)
    rospy.Subscriber("/pot_angle", AckermannDriveStamped, pot_angle,queue_size=1)
    rospy.spin()



if __name__ == '__main__':
    listener()

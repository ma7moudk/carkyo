#!/usr/bin/env python

'''
ackermann_drive_keyop.py:
    A ros keyboard teleoperation script for ackermann steering based robots
'''

__author__ = 'George Kouros'
__license__ = 'GPLv3'
__maintainer__ = 'George Kouros'
__email__ = 'gkourosg@yahoo.gr'

import roslib
import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float64
import sys, select, termios, tty
import thread ,time
from numpy import clip

old_steering=0.0
old_speed=0.0
last_time=time.time()
control_keys = {
    'up'    : '\x41',
    'down'  : '\x42',
    'right' : '\x43',
    'left'  : '\x44',
    'space' : '\x20',
    'n'     : '\x6E',
    'a'     : '\x61',
    'z'     : '\x7A',
    'x'     : '\x78',
    'tab'   : '\x09'}

key_bindings = {
    '\x41' : ( 1.0 , 0.0),
    '\x42' : (-1.0 , 0.0),
    '\x43' : ( 0.0 ,-1.0),
    '\x44' : ( 0.0 , 1.0),
    '\x6E' : ( 0.0 , 0.0),
    '\x61' : ( 0.0 , 0.0),
    '\x7A' : ( 0.0 , 0.0),
    '\x78' : ( 0.0 , 0.0),
    '\x20' : ( 0.0 , 0.0),
    '\x09' : ( 0.0 , 0.0)}


class AckermannDriveStampedKeyop:

    def __init__(self, args):
        if len(args) == 1:
            max_speed = float(args[0])
            max_steering_angle = float(args[0])
        elif len(args) >= 2:
            max_speed = float(args[0])
            max_steering_angle = float(args[1])
        else:
            max_speed = 3.0
            max_steering_angle = 0.35
            max_steering_angle_single = 0.04

        if len(args) > 2:
            cmd_topic = '/' + args[2]
        else:
            cmd_topic = 'rbcar_robot_control/command'

        self.speed_range = [-float(max_speed), float(max_speed)]
        self.steering_angle_range = [-float(max_steering_angle),
                                     float(max_steering_angle)]
        self.steering_angle_single_range = [-float(max_steering_angle_single),
                                     float(max_steering_angle_single)]
        for key in key_bindings:
            key_bindings[key] = \
                    (key_bindings[key][0] * float(max_speed) / 30,
                     key_bindings[key][1] * float(max_steering_angle) / 15)

        self.speed = 0
        self.steering_angle = 0
        self.steering_angle_velocity = 0
        self.jerk = 0
        self.motors_pub = rospy.Publisher(
            cmd_topic, AckermannDriveStamped, queue_size=1)
        rospy.Timer(rospy.Duration(0.1), self.pub_callback, oneshot=False)
        self.print_state()
        self.key_loop() 

    def pub_callback(self, event):
        ackermann_cmd_msg = AckermannDriveStamped()
        ackermann_cmd_msg.header.stamp=rospy.Time.now()
        ackermann_cmd_msg.drive.speed = self.speed
        ackermann_cmd_msg.drive.steering_angle = self.steering_angle
        ackermann_cmd_msg.drive.jerk = self.jerk
        self.motors_pub.publish(ackermann_cmd_msg)

    def print_state(self):
        sys.stderr.write('\x1b[2J\x1b[H')
        rospy.loginfo('\x1b[1M\r*********************************************')
        rospy.loginfo('\x1b[1M\rUse arrows to change speed and steering angle')
        rospy.loginfo('\x1b[1M\rUse space to brake and tab to align wheels')
        rospy.loginfo('\x1b[1M\rPress <ctrl-c> or <q> to exit')
        rospy.loginfo('\x1b[1M\r*********************************************')
##############################
        if (self.steering_angle == 0.70 or self.steering_angle == -0.70) and (old_steering == 0.70 or old_steering == -0.70)  :
            self.steering_angle = 0.00

 #       if (self.speed >= 0.44 and self.speed <= 0.46) and (old_speed >= -0.01 and old_speed <= 0.01)  :
  #          rospy.sleep(0.5)
   #     if (self.speed <= -0.44 and self.speed >= -0.46) and (old_speed >= -0.01 and old_speed <= 0.01)  :
    #        rospy.sleep(0.5)

################################
        rospy.loginfo('\x1b[1M\r'
                      '\033[34;1mSpeed: \033[32;1m%0.2f m/s, '
                      '\033[34;1mSteer Angle: \033[32;1m%0.2f rad\033[0m',
                      self.speed, self.steering_angle)

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        rospy.sleep(0.09)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key


    def key_loop(self):
        global old_steering , old_speed ,last_time
        self.settings = termios.tcgetattr(sys.stdin)
        while 1:
            key = self.get_key()
            current_time=time.time()
            if key in key_bindings.keys():
                if key == control_keys['space']:
                    self.speed = 0.0
                elif key == control_keys['tab']:
                    self.steering_angle = 0.0
                elif key == control_keys['a']:
                    self.steering_angle_velocity = 1
                elif key == control_keys['n']:
                    self.steering_angle_velocity = 0
                elif key == control_keys['z']:
                    self.jerk = 1
                elif key == control_keys['x']:
                    self.jerk = 0                    
                else:
                    self.speed = self.speed + key_bindings[key][0]
                    self.speed = clip(
                        self.speed, self.speed_range[0], self.speed_range[1])
					
                    if (current_time-last_time >=20.0):
		                self.steering_angle = \
		                        self.steering_angle + key_bindings[key][1]
		                self.steering_angle = clip(
		                    self.steering_angle,
		                    self.steering_angle_range[0],
		                    self.steering_angle_range[1])						
#		                    self.steering_angle_single_range[1])								                    
                    else:
		                self.steering_angle = \
		                        self.steering_angle + key_bindings[key][1]
		                self.steering_angle = clip(
		                    self.steering_angle,
		                    self.steering_angle_range[0],
		                    self.steering_angle_range[1])
                self.print_state()
            elif key == '\x03' or key == '\x71':  # ctr-c or q
                break
            else:
                continue

            old_steering = self.steering_angle
            old_speed = self.speed
            last_time=current_time
        self.finalize()

    def finalize(self):
        rospy.loginfo('Halting motors, aligning wheels and exiting...')
        self.settings = termios.tcgetattr(sys.stdin)
        ackermann_cmd_msg = AckermannDriveStamped()
        ackermann_cmd_msg.drive.speed = 0
        ackermann_cmd_msg.drive.steering_angle = 0
        ackermann_cmd_msg.drive.steering_angle_velocity = 0
        ackermann_cmd_msg.drive.jerk = 0
        self.motors_pub.publish(ackermann_cmd_msg)
        sys.exit()

if __name__ == '__main__':
    rospy.init_node('ackerm_drive_keyop_node')
    keyop = AckermannDriveStampedKeyop(sys.argv[1:len(sys.argv)])

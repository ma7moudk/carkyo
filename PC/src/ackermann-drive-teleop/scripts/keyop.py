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
import thread
from numpy import clip

control_keys = {
    'up'    : '\x41',
    'down'  : '\x42',
    'right' : '\x43',
    'left'  : '\x44',
    'space' : '\x20',
    'z' : '\x7A',
    'w'  : '\x77',
    's' : '\x73',
    'tab'   : '\x09'}

key_bindings = {
    '\x41' : ( 1.0 , 0.0 , 0.0),
    '\x42' : (-1.0 , 0.0 , 0.0),
    '\x43' : ( 0.0 ,-1.0 , 0.0),
    '\x44' : ( 0.0 , 1.0 , 0.0),
    '\x77' : ( 0.0 , 0.0 , 1.0),
    '\x73' : (0.0 , 0.0 , -1.0),
    '\x20' : ( 0.0 , 0.0 , 0.0),
    '\x7A' : ( 0.0 , 0.0 , 0.0),
    '\x09' : ( 0.0 , 0.0 , 0.0)}


class AckermannDriveStampedKeyop:

    def __init__(self, args):
        if True:
#        if len(args) == 1:
#            max_speed = float(args[0])
#            max_jerk = float(args[2])
#            max_steering_angle = float(args[0])
#        elif len(args) >= 2:
#            max_speed = float(args[0])
#            max_jerk = float(args[2])
#            max_steering_angle = float(args[1])
#        else:
            max_speed = 5
            max_jerk = 180
            max_steering_angle = 0.7

        if len(args) > 2:
            cmd_topic = '/' + args[2]
        else:
            cmd_topic = 'rbcar_robot_control/command'

        self.speed_range = [-float(max_speed), float(max_speed)]
        self.jerk_range = [-float(max_jerk), float(max_jerk)]
        self.steering_angle_range = [-float(max_steering_angle),
                                     float(max_steering_angle)]
        for key in key_bindings:
            key_bindings[key] = \
                    (key_bindings[key][0] * float(max_speed) / 5,
                     key_bindings[key][2] * float(max_jerk) / 180,
                     key_bindings[key][1] * float(max_steering_angle) / 5)

        self.speed = 0
        self.jerk = 0
        self.steering_angle = 0
        self.motors_pub = rospy.Publisher(
            cmd_topic, AckermannDriveStamped, queue_size=1)
        rospy.Timer(rospy.Duration(1.0/5.0), self.pub_callback, oneshot=False)
        self.print_state()
        self.key_loop()

    def pub_callback(self, event):
        ackermann_cmd_msg = AckermannDriveStamped()
        ackermann_cmd_msg.drive.speed = self.speed
        ackermann_cmd_msg.drive.jerk = self.jerk
        ackermann_cmd_msg.drive.steering_angle = 0.0
        self.motors_pub.publish(ackermann_cmd_msg)

    def print_state(self):
        sys.stderr.write('\x1b[2J\x1b[H')
        rospy.loginfo('\x1b[1M\r*********************************************')
        rospy.loginfo('\x1b[1M\rUse arrows to change speed and steering angle')
        rospy.loginfo('\x1b[1M\rUse space to brake and tab to align wheels')
        rospy.loginfo('\x1b[1M\rPress <ctrl-c> or <q> to exit')
        rospy.loginfo('\x1b[1M\r*********************************************')
        rospy.loginfo('\x1b[1M\r'
                      '\033[34;1mSpeed: \033[32;1m%0.2f m/s, '
                      '\033[34;1mSteer Angle: \033[32;1m%0.2f rad\033[0m, '
                      '\033[34;1mJerk: \033[32;1m%0.2f deg ',
                      self.speed, self.steering_angle , self.jerk)

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key


    def key_loop(self):
        self.settings = termios.tcgetattr(sys.stdin)
        while 1:
            key = self.get_key()
            if key in key_bindings.keys():
                if key == control_keys['space']:
                    self.speed = 0.0
                elif key == control_keys['tab']:
                    self.steering_angle = 0.0
                elif key == control_keys['z']:
                    self.jerk = 0.0
                else:
                    self.speed = self.speed + key_bindings[key][0]
                    self.jerk = self.jerk + key_bindings[key][2]
                    self.steering_angle = \
                            self.steering_angle + key_bindings[key][1]
                    self.speed = clip(
                        self.speed, self.speed_range[0], self.speed_range[1])
                    self.jerk = clip(
                        self.jerk, self.jerk_range[0], self.jerk_range[1])
                    self.steering_angle = clip(
                        self.steering_angle,
                        self.steering_angle_range[0],
                        self.steering_angle_range[1])
                self.print_state()
            elif key == '\x03' or key == '\x71':  # ctr-c or q
                break
            else:
                continue
        self.finalize()

    def finalize(self):
        rospy.loginfo('Halting motors, aligning wheels and exiting...')
        self.settings = termios.tcgetattr(sys.stdin)
        ackermann_cmd_msg = AckermannDriveStamped()
        ackermann_cmd_msg.drive.speed = 0
        ackermann_cmd_msg.drive.steering_angle = 0
        ackermann_cmd_msg.drive.jerk = 0
        self.motors_pub.publish(ackermann_cmd_msg)
        sys.exit()

if __name__ == '__main__':
    rospy.init_node('ackermann_drive_keyop_node')
    keyop = AckermannDriveStampedKeyop(sys.argv[1:len(sys.argv)])

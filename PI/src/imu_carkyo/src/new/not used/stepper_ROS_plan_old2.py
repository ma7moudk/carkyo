#!/usr/bin/env python
import rospy , math , time 
from ackermann_msgs.msg import AckermannDriveStamped
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BOARD)
stepper_pos=0
old_steering_angle=0.0
angle_val=0.0 #received angle from pot node
enable =1 # enable stepper change only when angle_val is between -30 , +30

rospy.init_node('steering_node', anonymous=True)
direction = 12
GPIO.setup(10,GPIO.OUT)
GPIO.setup(8,GPIO.OUT)
GPIO.setup(direction,GPIO.OUT)
print "starting"

def callback_absolute(data):
    global stepper_pos ,old_steering_angle
    
    steering_angle=data.drive.steering_angle
    allign=data.drive.steering_angle_velocity
         
    if (steering_angle != old_steering_angle):
        steeringTemp=steering_angle
        steeringTemp=translate(steeringTemp, -0.70, 0.70, -25, 25)
        steeringTemp=normalize(steeringTemp,-25,25)
        rospy.loginfo("changing")
        stepper_change(int(steeringTemp))
        print "steeringTemp,steering_angle" , steeringTemp,steering_angle

    old_steering_angle=steering_angle


def normalize(val,min_val,max_val):
    if val <=min_val:
        val=min_val
    if val >=max_val:
        val=max_val
    return val
        
    
def stepper_change(i):
    global stepper_pos , direction ,angle_val ,enable
    print "Num of steps : ",i
#    stepper_pos = stepper_pos + i
####################################################
    i=i-stepper_pos
    stepper_pos = stepper_pos + i
#oldpos gv abs_i rltv_i abs_pos
#pos =0 give i=+2 i=+2  pos=+2
#pos=+2 give i=+3 i=+1  pos=+3
#pos=+3 give i=-5 i=-8  pos=-5

####################################################
    if(angle_val>14.0) or(angle_val<-14.0):
        enable=0
    else:
        enable=1

    if i < 0:
        GPIO.output(8,0)
        GPIO.output(direction,1)
        i = i*-1
    elif i > 0:
        GPIO.output(8,0)
        GPIO.output(direction,0)
        
    if enable == 1:
        for x in range(0,i*20):
            GPIO.output(10,1)
            time.sleep(2.0/1000.0)
            GPIO.output(10,0)
            time.sleep(2.0/1000.0)
        GPIO.output(8,1)

def callback_relative(data):
    global stepper_pos ,old_steering_angle
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
            steeringTemp=translate(steeringTemp, -0.70, 0.02, -15, -1)
            rospy.loginfo("1st condition more negative")
            stepper_change_relative(int(steeringTemp))

        elif steering_angle < 0.0 and (steering_angle > old_steering_angle) :
            steeringTemp=steering_angle
            steeringTemp=translate(steeringTemp, -0.70, 0.02, 6, 1)
            rospy.loginfo("2nd condition less negative")
            stepper_change_relative(int(steeringTemp))

        elif steering_angle > 0.0 and (steering_angle > old_steering_angle):
            steeringTemp=steering_angle
            steeringTemp=translate(steeringTemp, 0.02, 0.70,1, 15)
            rospy.loginfo("3rd condition more positive")
            stepper_change_relative(int(steeringTemp))

        elif steering_angle > 0.0 and (steering_angle < old_steering_angle):
            steeringTemp=steering_angle
            steeringTemp=translate(steeringTemp, 0.02, 0.70, -1, -6)
            rospy.loginfo("4th condition less positive ")
            stepper_change_relative(int(steeringTemp))
          
    old_steering_angle=steering_angle


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
 
def callback_angle(data): #add odommsg as global
    global angle_val
    angle_val=data.drive.steering_angle
   

def listener():
    rospy.Subscriber("/rbcar_robot_control/command", AckermannDriveStamped, callback_absolute,queue_size=1)
    rospy.Subscriber("/rbcar_robot_control/command1", AckermannDriveStamped, callback_relative,queue_size=1)
    rospy.Subscriber("/pot_angle", AckermannDriveStamped, callback_angle,queue_size=1)
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

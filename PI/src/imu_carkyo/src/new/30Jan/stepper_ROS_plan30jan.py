#!/usr/bin/env python
import rospy , math , time 
from ackermann_msgs.msg import AckermannDriveStamped
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BOARD)
stepper_pos=0
old_steering_angle=0.0

steps_sign=1
steps=0

angle_val=0.0 #received angle from pot node
enable =1 # enable stepper change only when angle_val is between -30 , +30  *0.26  0.26 -14 14 

rospy.init_node('steering_node', anonymous=True)
direction = 12
GPIO.setup(10,GPIO.OUT)
GPIO.setup(8,GPIO.OUT)
GPIO.setup(direction,GPIO.OUT)
print "starting"
steering_angle=0.0
enable_angle_feedback=0

def callback_absolute(data):
    global steering_angle , enable_angle_feedback
    steering_angle=data.drive.steering_angle
    enable_angle_feedback=1
    
    

def timeout_fun(event):
    global enable_angle_feedback
    if (enable_angle_feedback == 1 ):
        enable_angle_feedback=0
    else:
        print "delayed callback"

rospy.Timer(rospy.Duration(1.0),timeout_fun)


def normalize(val,min_val,max_val):
    if val <=min_val:
        val=min_val
    if val >=max_val:
        val=max_val
    return val
        
    
def stepper_change():
    global direction ,angle_val ,enable,steering_angle , steps_sign , steps
    steering_angle_deg=steering_angle *180.0/math.pi
    print "req ang: ",steering_angle_deg , "curr ang" , angle_val

    if(steering_angle_deg>14.0) :
        steering_angle_deg=14.0
    elif(steering_angle_deg<-14.0):
        steering_angle_deg=-14.0

    angle=steering_angle_deg-angle_val
    steps=int(angle *2)


    if steps<0:
        enable=1
        GPIO.output(8,0)
        GPIO.output(direction,1)
    elif steps>0:
        enable=1    
        GPIO.output(8,0)
        GPIO.output(direction,0) 
    else:
        enable =0
                    
    steps =abs(steps)

#-ve is right is direction 1


        
    if enable == 1 and steps >= 0.05:
        for x in range(0,steps*20):
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

    for x in range(0,i*15):
        GPIO.output(10,1)
        time.sleep(2.0/1000.0)
        GPIO.output(10,0)
        time.sleep(2.0/1000.0)
    GPIO.output(8,1)
 
last_time = rospy.Time.now()
def callback_angle(data): #add odommsg as global
    global angle_val , last_time,enable_angle_feedback
    angle_val=data.drive.steering_angle
    if (data.header.stamp>last_time):
        if (enable_angle_feedback==1):
            stepper_change()
       # print "topic vs now" , data.header.stamp , rospy.Time.now()
        last_time = rospy.Time.now()
   

def listener():
    rospy.Subscriber("/rbcar_robot_control/command", AckermannDriveStamped, callback_absolute,queue_size=1)
    rospy.Subscriber("/rbcar_robot_control/command1", AckermannDriveStamped, callback_relative,queue_size=1)
    rospy.Subscriber("/pot_angle", AckermannDriveStamped, callback_angle,queue_size=1)
    rospy.spin()

def translate(value, leftMin, leftMax, rightMin, rightMax):
    leftSpan = leftMax - leftMin
    rightSpan = rightMax - rightMin
    valueScaled = float(value - leftMin) / float(leftSpan)
    return rightMin + (valueScaled * rightSpan)

if __name__ == '__main__':
    listener()

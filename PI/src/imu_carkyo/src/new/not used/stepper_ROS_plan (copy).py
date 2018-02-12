#!/usr/bin/env python
import rospy , math , time ,datetime
from ackermann_msgs.msg import AckermannDriveStamped
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BOARD)
angle_val = 0.0	#received angle from pot node
enable = 1		# enable stepper change only when angle_val is between -30 , +30  *0.26  0.26 -14 14 

rospy.init_node('steering_node', anonymous = True)
direction = 12
GPIO.setup(10, GPIO.OUT)
GPIO.setup(8, GPIO.OUT)
GPIO.setup(direction, GPIO.OUT)
print "starting"
steering_angle = 0.0
enable_angle_feedback = 0

seq = 0

def callback_absolute(data):
	global steering_angle , enable_angle_feedback, seq
	steering_angle=data.drive.steering_angle
	seq = data.header.seq
	enable_angle_feedback = 1

def timeout_fun(event):
	global enable_angle_feedback
	if (enable_angle_feedback == 1 ):
		enable_angle_feedback = 0
	else:
		print "delayed callback"

rospy.Timer(rospy.Duration(1.0), timeout_fun)

def normalize(val,min_val,max_val):
	if val <= min_val:
		val = min_val
	if val >= max_val:
		val = max_val
	return val

def stepper_change():
    global direction ,angle_val ,enable,steering_angle , steps_sign , steps
    steering_angle_deg = steering_angle*180.0/math.pi
    #print "req ang: ",steering_angle_deg , "curr ang" , angle_val
    if (steering_angle_deg > 14.0):
        steering_angle_deg = 14.0
    elif (steering_angle_deg < -14.0):
        steering_angle_deg = -14.0
    angle = steering_angle_deg - angle_val
    DIR = ""
    if angle < 0:
        enable = 1
        GPIO.output(8,0)
        GPIO.output(direction,1)
        DIR = "RIGHT"
    elif angle > 0:
        enable = 1
        GPIO.output(8,0)
        GPIO.output(direction,0)
        DIR = "LEFT"
    else:
        enable = 0
    #-ve is right is direction 1
    if enable == 1 and abs(angle) >= 0.5:
        with open(str(datetime.datetime.now().month)+"-"+str(datetime.datetime.now().day)+"-"+str(datetime.datetime.now().hour)+"-Stepper.csv", "a") as text_file:
            text_file.write("{0},-,{1},{2},-,{3}\n".format(seq,steering_angle *180.0/math.pi,angle_val,DIR))
        for x in range(0,10):
            GPIO.output(10,1)
            time.sleep(2.0/1000.0)
            GPIO.output(10,0)
            time.sleep(2.0/1000.0)
        GPIO.output(8,1)
last_time=rospy.Time.now()
def callback_angle(data): #add odommsg as global
    global angle_val , last_time,enable_angle_feedback
    angle_val=data.drive.steering_angle
    if (data.header.stamp > last_time):
        if (enable_angle_feedback == 1):
            stepper_change()
       # print "topic vs now" , data.header.stamp , rospy.Time.now()
        last_time = rospy.Time.now()

def listener():
    rospy.Subscriber("/rbcar_robot_control/command", AckermannDriveStamped, callback_absolute,queue_size=1)
    rospy.Subscriber("/pot_angle", AckermannDriveStamped, callback_angle,queue_size=1)
    rospy.spin()

def translate(value, leftMin, leftMax, rightMin, rightMax):
    leftSpan = leftMax - leftMin
    rightSpan = rightMax - rightMin
    valueScaled = float(value - leftMin) / float(leftSpan)
    return rightMin + (valueScaled * rightSpan)

if __name__ == '__main__':
    listener()

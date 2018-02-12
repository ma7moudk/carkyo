#!/usr/bin/env python
import rospy ,math
import time 
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BOARD)

modePins = (36,40,38)
accSwitch=35
brakeSwitch=37
accPins   = (33,32,31,29,26,24)
breakPins = (23,22,21,19,18,16)
GPIO.setup(modePins,GPIO.OUT) 
GPIO.setup(accSwitch,GPIO.OUT) 
GPIO.setup(brakeSwitch,GPIO.OUT) 
GPIO.setup(accPins,GPIO.OUT)	
GPIO.setup(breakPins,GPIO.OUT)

changing=0
started=0

lengthBetweenTwoWheels=1.05
wheelBase=1.65

angle_val=0.0
old_speed=0.0
speedTemp=0.0
mode_time=time.time()

odom_pub = rospy.Publisher('/wheel_odometry', Odometry, queue_size=10)
odomMsg = Odometry()


def set_mode_normal():
	global mode, modePins
	modeVal = (0, 1, 1) ##ma7moudk23Oct
	GPIO.output(modePins, modeVal)
	mode = "Normal"
	print ("mode = Normal")

def set_mode_forward():
	global mode, modePins
	modeVal = (0, 0, 1) ##ma7moudk23Oct
	GPIO.output(modePins, modeVal)
	mode = "Forward"

def set_mode_reverse():
	global mode, modePins
	modeVal = (0, 1, 0) ##ma7moudk23Oct
	GPIO.output(modePins, modeVal)
	mode = "Reverse"

def set_mode_off():
	global mode, modePins
	modeVal = (1, 1, 1) ##ma7moudk23Oct
	GPIO.output(modePins, modeVal)
	mode = "Off"

def accelerate(accVal):
	global accSwitch,  accPins#ma7moudk23Oct add
	if accVal <= 12 :
		GPIO.output(accSwitch, 1)
	else:
		GPIO.output(accSwitch, 0)

	accVal = list("{0:b}".format(int(accVal)))
	while len(accVal)<6:
		accVal = ['0']+accVal
	accVal = map(int,accVal)
	GPIO.output(accPins,accVal)
	print len(accVal),accVal,"Acceleration"

def brake(brakeVal):	
	brakeVal = list("{0:b}".format(brakeVal))
	while len(brakeVal)<6:
		brakeVal = ['0']+brakeVal
	brakeVal = map(int,brakeVal)
	GPIO.output(breakPins,brakeVal)
	#print len(brakeVal),brakeVal

def change_mode(newMode):
    global mode, changing, started


#    open ("/home/pi/change_mode.txt", "w+").write("ChangeModeCalled")
    if changing == 1:
        return
    if started == 0:
        changing = 1
        print "Starting Mode: ", newMode
        set_mode_off()
        time.sleep(2)
        accelerate(14)  # 0.6
        brake(10)  # 0.5 volts
        if newMode == "Normal":
            set_mode_normal()
            print "Set Mode: ", newMode
        elif newMode == "Forward":
            set_mode_forward()
        elif newMode == "Reverse":
            set_mode_reverse()
        elif newMode == "Off":
            set_mode_off()
        time.sleep(2.5)
        print("after")
        accelerate(12)  # 0.5 volts
        time.sleep(4)
        if newMode == "Off":
            started = 0
        else:
            started = 1
        changing = 0
    else:
        if newMode == "Normal":
            set_mode_normal()
        elif newMode == "Forward":
            set_mode_forward()
        elif newMode == "Reverse":
            set_mode_reverse()
        elif newMode == "Off":
            set_mode_off()
#####$$$$$##### keyboard callback #####$$$$$##### 
def callback(data):
    global old_speed ,mode_time,speedTemp
    speed=data.drive.speed

    if (speed != old_speed):
        if (speed >= -0.1) and (speed <= 0.1)  : ### Case 0
            speedTemp=13
            change_mode("Normal")

        elif speed <= -0.1 : ##### Case 3
            if (mode != "Reverse"):
                rospy.loginfo("mode wasn't backward ")
                change_mode("Reverse")
                mode_time=time.time()
            speedTemp=speed
            speedTemp=translate(speedTemp, -5.00, -0.90, 63, 22)

        elif speed >= 0.1 : ##### Case 4
            if (mode !="Forward"):
                rospy.loginfo("mode wasn't forward ")
                change_mode("Forward")
                mode_time=time.time()
            speedTemp=speed
            speedTemp=translate(speedTemp, 0.90, 5.00, 22, 63) 

        else : ##### Case 5
            print "else"     

        if(time.time()-mode_time>1.0):
			accelerate(int(speedTemp))
          
    old_speed=speed

def step2fBetaRads():
    global fBetaRads,current_step ,current_sign
    fBetaRads=current_sign*current_step*2
    fBetaRads=fBetaRads*math.pi/180.0

####################################################################3
def callback_speed(data): #add odommsg as global
    global mode ,lengthBetweenTwoWheels ,wheelBase ,angle_val
    left=data.twist.twist.linear.x
    right=data.twist.twist.linear.y
    left_sensor=data.pose.pose.position.x
    right_sensor=data.pose.pose.position.y
    if mode=="Reverse":
        vL= -left
        vR= -right
        vL_sensor= -left_sensor
        vR_sensor= -right_sensor

    elif mode=="Forward":
        vL= left
        vR= right
        vL_sensor= left_sensor
        vR_sensor= right_sensor

    else: #change to zero
        vL= left
        vR= right
        vL_sensor= left_sensor
        vR_sensor= right_sensor

    v_mps= (vR + vL)/ 2.0
#    velocity_theta=(vR - vL)/ lengthBetweenTwoWheels
#    if velocity_theta==0:
#        wheelAngle=0.0
#    else:
#        radiusOfRotation=v_mps/velocity_theta #ma7moudk25oct
#        wheelAngle=math.atan(wheelBase/radiusOfRotation)
    wheelAngle=angle_val
    vx = v_mps * math.cos(wheelAngle)
    vy = v_mps * math.sin(wheelAngle)
    odomMsg.twist.twist.linear.x = vx
    odomMsg.twist.twist.linear.y = vy
    odomMsg.twist.twist.linear.z = 0.0  
    odomMsg.header.frame_id = 'odom'
    odomMsg.child_frame_id = 'base_footprint'
    odomMsg.twist.covariance = [0.0001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1000, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1000, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1000, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1000]
    odomMsg.pose.covariance = [0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    odomMsg.header.stamp= rospy.Time.now()
    odom_pub.publish(odomMsg)

def callback_angle(data): #add odommsg as global
    global angle_val
    angle_value=data.drive.steering_angle

def listener():
    rospy.init_node('Control_node', anonymous=True)
    rospy.Subscriber("/rbcar_robot_control/command", AckermannDriveStamped, callback,queue_size=1)
    rospy.Subscriber("/angle_value", AckermannDriveStamped, callback_angle,queue_size=1)
    rospy.Subscriber("/speed_values", Odometry, callback_speed,queue_size=1)
    rospy.spin()

def translate(value, leftMin, leftMax, rightMin, rightMax):
    # Figure out how 'wide' each range is
    leftSpan = leftMax - leftMin
    rightSpan = rightMax - rightMin
    # Convert the left range into a 0-1 range (float)
    valueScaled = float(value - leftMin) / float(leftSpan)
    # Convert the 0-1 range into a value in the right range.
    return rightMin + (valueScaled * rightSpan)

change_mode("Normal")
if __name__ == '__main__':
    listener()

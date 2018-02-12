#!/usr/bin/env python
# Accepts motion commands and send it to HW
# Receives HW status: Wheel odom from HW and car controller
##ma7moudk23Oct : seprate mode pins out of switches pins .. Switches value depends on DAC value
#####$$$$$##### Includes #####$$$$$##### 
import rospy ,math
import time 
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import Imu
import RPi.GPIO as GPIO
from threading import Thread
GPIO.setmode(GPIO.BOARD)
#from encoder_test_2 import *
import encoder_test_2
#####$$$$$##Global variables##$$$$$##### 
print encoder_test_2.SubSteps
yaw=0.0
ang_x=0.0
ang_y=0.0
ang_z=0.0
fBetaRads=0.0
allign=0
vL=0.0
vR=0.0
dt=0.0
mode_time=time.time()
tickR= 0.0 
tickL= 0.0
pos2_x=0.0
pos2_y=0.0
pos1_x=0.0
pos1_y=0.0
th =0.0
t=0.0
ticksPerSpin = 52
diameter = 0.41
lengthBetweenTwoWheels = 1.01
wheelBase=1.65 #ma7moudk25oct
DistancePerCount = (math.pi * diameter) / ticksPerSpin
steering_angle=0
speed=0
old_steering_angle=0.0
old_speed=10.0
current_step=0
current_sign=0
vx=0.0
vy=0.0
velocity_x=0.0
velocity_theta=0.0
prevTime=0.0
prevTickR =0.0
prevTickL = 0.0
mode="Off"
speedTemp=0.0
changing = 0
started = 0

encoder_test_2.ResetStepperPos()
###############################################################
###Global values for encoders######
NumberOfInterrputsPorts = 4   # 4 I/Os
TEN_MICRO = 0.0001 # temp = 1 sec (10.0 / 1000.0) / (1000.0)        # units in sec
TenMicros = 0
Millies = 0
Secs = 0
Minutes = 0
TimingError = 0
CorrectTiming = 0
WaitCounter = 0

Prev = [0,0,0,0]
consecutivezeros = [0,0,0,0]
consecutiveones = [0,0,0,0]
threshold = [5,5,1,1]
Falling = [0,0,0,0]
Rising = [0,0,0,0]
Transition = [0,0,0,0]
pin=[13,15,7,11]

wheelAngle=0.0

# >>> Interrupts configurations and functions ..
#interruptPins = (7,8,10,12)

#GPIO.setup(interruptPins,GPIO.IN)
GPIO.setup(pin,GPIO.IN,pull_up_down=GPIO.PUD_UP)



print "before ros init"
rospy.init_node('wheel_odometry_node', anonymous=True)
print "after ros init"

print "after thread"
#print time.time()
##while (True):
 #   print "Time: ", TimingError, CorrectTiming, Rising[0], Rising[1], Rising[2], Rising[3], TEN_MICRO, WaitCounter, time.clock()
    #time.sleep(5)
    #print "Time: ", TimingError, CorrectTiming, Minutes, Secs, Millies, TEN_MICRO, WaitCounter, time.clock()

#            n, f, r, a, b
#            y, g, b, p,gray
#modePins = (36,40,38,35,37)
#GPIO.setup(modePins,GPIO.OUT)
modePins = (36,40,38)  ##ma7moudk23Oct
accSwitch=35 ##ma7moudk23Oct
brakeSwitch=37 ##ma7moudk23Oct
GPIO.setup(modePins,GPIO.OUT) ##ma7moudk23Oct
GPIO.setup(35,GPIO.OUT) ##ma7moudk23Oct
GPIO.setup(37,GPIO.OUT) ##ma7moudk23Oct


accPins   = (33,32,31,29,26,24)
breakPins = (23,22,21,19,18,16)
GPIO.setup(accPins,GPIO.OUT)	
GPIO.setup(breakPins,GPIO.OUT)


# >>> Stepper Pins and Functions ..
stepperPins = (8,10,12)
#stepper_pos = 0
encoder_test_2.ResetStepperPos()

GPIO.setup(stepperPins,GPIO.OUT)




def getWheelAngle():
    global wheelAngle
    return wheelAngle


def set_mode_normal():
	global mode, modePins
	#modeVal = (0, 1, 1, 1, 0)
	modeVal = (0, 1, 1) ##ma7moudk23Oct
	GPIO.output(modePins, modeVal)
	mode = "Normal"

def set_mode_forward():
	global mode, modePins
	#modeVal = (0, 0, 1, 0, 1)
	modeVal = (0, 0, 1) ##ma7moudk23Oct
	GPIO.output(modePins, modeVal)
	mode = "Forward"

def set_mode_reverse():
	global mode, modePins
	#modeVal = (0, 1, 0, 0, 1)
	modeVal = (0, 1, 0) ##ma7moudk23Oct
	GPIO.output(modePins, modeVal)
	mode = "Reverse"

def set_mode_off():
	global mode, modePins
	#modeVal = (1, 1, 1, 1, 0)
	modeVal = (1, 1, 1) ##ma7moudk23Oct
	GPIO.output(modePins, modeVal)
	mode = "Off"

def switch_acc(val):
	GPIO.output(35, val)


# >>> function to convert acc value to binary and apply it to pins >>>
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


# >>> function to convert breaks value to binary and apply it to pins >>>
def brake(brakeVal):
	
	brakeVal = list("{0:b}".format(brakeVal))
	while len(brakeVal)<6:
		brakeVal = ['0']+brakeVal
	brakeVal = map(int,brakeVal)
	GPIO.output(breakPins,brakeVal)
	#print len(brakeVal),brakeVal

# >>> function to change bet. modes >>>
def change_mode(newMode):
    global mode, changing, started

    if changing == 1:
        return

    if started == 0:
        changing = 1
        print "Starting Mode: ", newMode
        set_mode_off()
        time.sleep(2)
#66 49
#61 51
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
        # waits for 2 sec
        # rospy.Timer(rospy.Duration(2), callback2sec, oneshot=True)
        time.sleep(4) ##ma7moudk23Oct commented
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

def callback2sec(event):
    accelerate(10)  # 0.5 volts
  #  switch_acc(1) ##ma7moudk23Oct commented


#####$$$$$##### ROS init #####$$$$$##### 

odom_pub = rospy.Publisher('/wheel_odometry', Odometry, queue_size=10)
odomMsg = Odometry()

odomMsg.twist.covariance = [0.0001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1000, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1000, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1000, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1000]

odomMsg.pose.covariance = [0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


change_mode("Normal")

#####$$$$$##### keyboard callback #####$$$$$##### 
def callback(data):
    global mode_time, speedTemp, steering_angle,speed,old_steering_angle,old_speed ,mode,angle,acc, current_step ,current_sign
    global TimingError, CorrectTiming, Rising, TEN_MICRO, WaitCounter
    print"--------------------------------------------"
    print "Time: ", TimingError, CorrectTiming, Rising[0], Rising[1], Rising[2], Rising[3], TEN_MICRO, WaitCounter, time.clock()
    print"--------------------------------------------"
    print encoder_test_2.GetStepperPos() , getWheelAngle()
    steering_angle=data.drive.steering_angle
    speed=data.drive.speed
    allign=data.drive.steering_angle_velocity

	
#    if (allign == 1.0)  : 
#        rospy.loginfo('\x1b[1M\r'
#                    '\033[34;1mAlligned: \033[32;1m%0.2f ',
#                    allign)
#        stepper_pos =0
#    elif (allign == 0.0)  : 
 #       rospy.loginfo('\x1b[1M\r'
 #                 '\033[34;1mAlligned: \033[32;1m%0.2f ',
 #                 allign)

    if (speed >= -0.02) and (speed <= 0.02)  : ##### Case 0
        change_mode("Normal")
        accelerate(10)

    if (speed != old_speed):
        if (speed < -0.02 ) and (speed > -0.90) :  ##### Case 1
            print "BackMode jvlkfbcn  lmh yhk ;tyjm yujm " , mode
            if (mode != "Reverse"):
                rospy.loginfo("mode wasn't backward ")
                change_mode("Reverse")
                mode_time=time.time()

        elif (speed > 0.02 ) and (speed < 0.90) :  ##### Case 2
            if (mode !="Forward"):
                rospy.loginfo("mode wasn't forward ")
                change_mode("Forward")
                mode_time=time.time()

        elif speed <= -0.905 : ##### Case 3
            speedTemp=speed
            speedTemp=translate(speedTemp, -5.00, -0.90, 63, 12)

        elif speed >= 0.905 : ##### Case 4
            speedTemp=speed
            speedTemp=translate(speedTemp, 0.90, 5.00, 12, 63) 

        else : ##### Case 5
            print "else"     

        if(time.time()-mode_time>1.0):
			accelerate(int(speedTemp))

            #########steering#############
         
    if (steering_angle != old_steering_angle):
        if steering_angle < 0.0 and (steering_angle < old_steering_angle) :
            steeringTemp=steering_angle
            steeringTemp=translate(steeringTemp, -0.70, 0.00, -15, 0)
            rospy.loginfo("1st condition more negative")
            encoder_test_2.stepper_change(int(steeringTemp))

        elif steering_angle < 0.0 and (steering_angle > old_steering_angle) :
            steeringTemp=steering_angle
            steeringTemp=translate(steeringTemp, -0.70, 0.00, 8, 0)
            rospy.loginfo("2nd condition less negative")
            encoder_test_2.stepper_change(int(steeringTemp))

        elif steering_angle > 0.0 and (steering_angle > old_steering_angle):
            steeringTemp=steering_angle
            steeringTemp=translate(steeringTemp, 0.00, 0.70, 0, 15)
            rospy.loginfo("3rd condition more positive")
            encoder_test_2.stepper_change(int(steeringTemp))

        elif steering_angle > 0.0 and (steering_angle < old_steering_angle):
            steeringTemp=steering_angle
            steeringTemp=translate(steeringTemp, 0.00, 0.70, 0, -8)
            rospy.loginfo("4th condition less positive ")
            encoder_test_2.stepper_change(int(steeringTemp))
          
    old_steering_angle=steering_angle
    old_speed=speed

def step2fBetaRads():
    global fBetaRads,current_step ,current_sign
    fBetaRads=current_sign*current_step*2
    fBetaRads=fBetaRads*math.pi/180.0

####################################################################3
def pub_callback(event): #add odommsg as global
    global velocity_theta , velocity_x , odomMsg , odom2Msg,odom_pub ,odom2_pub , pos2_x , pos2_y, pos1_x , pos1_y, stepper_pos
    global current_time , th ,ticksPerSpin , diameter ,lengthBetweenTwoWheels ,DistancePerCount, vx ,vy , vL ,vR , dt  
    global current_sign
    global yaw , ang_x , ang_y , ang_z ,fBetaRads,current_step 
    global 	prevTime , prevTickR , prevTickL 
    global wheelAngle
    tickR = Rising[0]
    tickL = Rising[1]
    current_step = encoder_test_2.GetStepperPos()
    step2fBetaRads()
    if current_step < 0.0:
        current_sign = -1
        print "........................pointing Right"
    elif current_step > 0.0:
        current_sign = 1
        print "........................pointing Left"
    current_time = time.time()
    deltaL = tickL- prevTickL
    deltaR = tickR- prevTickR
    dt = current_time - prevTime

    if mode=="Reverse":

        vL= -(deltaL * DistancePerCount)/dt
        vR= -(deltaR * DistancePerCount)/dt

    elif mode=="Forward":

        vL= (deltaL * DistancePerCount)/dt
        vR= (deltaR * DistancePerCount)/dt

    else:
        #  vL= 0 #commented only for debugging should be 0
        # vR= 0
        vL= (deltaL * DistancePerCount)/dt
        vR= (deltaR * DistancePerCount)/dt

    v_mps= (vR + vL)/ 2.0
    vx = v_mps * math.cos(fBetaRads) #* math.cos(yaw)  #fBetaRads is trurning angle , robot_pose_pa_ is yaw
    vy = v_mps * math.sin(fBetaRads) #* math.sin(yaw)
    pos1_x += vx *dt
    pos1_y += vy *dt
    ######### calc angle ##################### #ma7moudk25oct
    velocity_theta=(vR - vL)/ lengthBetweenTwoWheels #ma7moudk25oct
    if velocity_theta==0:
        wheelAngle=0.0
    else:
        radiusOfRotation=v_mps/velocity_theta #ma7moudk25oct
        wheelAngle=math.atan(wheelBase/radiusOfRotation) #ma7moudk25oct
#####Publish ROS message############
    odomMsg.pose.pose.position.x = pos1_x
    odomMsg.pose.pose.position.y = pos1_y
    odomMsg.twist.twist.linear.x = vx
    odomMsg.twist.twist.linear.y = vy
    odomMsg.twist.twist.linear.z = 0.0  
    odomMsg.header.frame_id = 'odom'
    odomMsg.child_frame_id = 'base_footprint'
    odomMsg.header.stamp= rospy.Time.now()
    odom_pub.publish(odomMsg)
    prevTime=current_time
    prevTickR=tickR
    prevTickL=tickL

rospy.Timer(rospy.Duration(0.1), pub_callback)


def callbackImu(data):
    global yaw , ang_x , ang_y , ang_z
    quat_x=data.orientation.x
    quat_y=data.orientation.y
    quat_z=data.orientation.z
    quat_w=data.orientation.w
    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion([quat_x, quat_y, quat_z, quat_w])
    ang_x=data.angular_velocity.x
    ang_y=data.angular_velocity.y
    ang_z=data.angular_velocity.z       
    #print "Callback IMU -- yaw: " , yaw*180.0/3.14



def listener():
    rospy.init_node('wheel_odometry_node', anonymous=True)
    #print "..........stating Listener........."
    rospy.Subscriber("/rbcar_robot_control/command", AckermannDriveStamped, callback,queue_size=1)
    rospy.Subscriber("/imu_enu_new",Imu, callbackImu,queue_size=2)
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

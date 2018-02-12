#!/usr/bin/env python
import rospy ,math
import time ,tf
import numpy as np
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BOARD)

pitch=0.0 # global variable @ callback_imu , callback_mover_base
a=1.5   # parameter to increase wheel odometry average speed (to make distance is closer to reality)
## PI GPIO configrations
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

jerk=1
#global variables for timeout
timeout_command_flag=0
timeout_command_flag_key=0

speed=0.0
#global variables for change mode
changing=0
started=0
m=25.64
b=25.64
#global variables for odometry
lengthBetweenTwoWheels=1.05
wheelBase=1.65
v_mps=0.0
angle_val=0.0
old_speed=0.0
speedTemp=0.0
speedTempAcc=0.0
kp= 0.7  # PID control for speed

mode_time=time.time()
old_speed1=0.0
speedTemp1=0.0
speedTemp1Acc=0.0

    
speed_counter=1
Vmps_10=[0.0,0.0,0,0,0,0,0,0,0,0]

rospy.init_node('Control_node', anonymous=True)
mode_time1=time.time()
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
#	print("accVal",accVal)
	if accVal <= 12 :
		GPIO.output(accSwitch, 1)
	else:
		GPIO.output(accSwitch, 0)

	accVal = list("{0:b}".format(int(accVal)))
	while len(accVal)<6:
		accVal = ['0']+accVal
	accVal = map(int,accVal)
	GPIO.output(accPins,accVal)
#	print accVal,"DAC pins"

def brake(brakeVal):	
	brakeVal = list("{0:b}".format(brakeVal))
	while len(brakeVal)<6:
		brakeVal = ['0']+brakeVal
	brakeVal = map(int,brakeVal)
	GPIO.output(breakPins,brakeVal)
	#print len(brakeVal),brakeVal

def change_mode(newMode):
    global mode, changing, started
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
#####$$$$$##### timeout fun#####$$$$$##### 

def timeout_fun(event):
    global timeout_command_flag , jerk
    if (timeout_command_flag == 0 ):
        timeout_command_flag=1
    else:
        print "delayed callback"
        if jerk == 1:
            print "disabling move base last command"
            accelerate(13)
                 
rospy.Timer(rospy.Duration(1.0),timeout_fun)           
#####$$$$$##### move_base callback #####$$$$$##### 
def callback_move_base(data):
    global timeout_command_flag ,speed , Vmps_10  , pitch ,kp
    timeout_command_flag =0
    global old_speed ,mode_time,speedTemp,speedTempAcc,speed_counter
    speed=data.drive.speed

    if ((speed != old_speed):
        speed_mod = speed
        
    if ((speed != old_speed) or (Vmps_10[speed_counter] != Vmps_10[speed_counter-1]  ) ):
        if (speed >= -0.05) and (speed <= 0.05)  : ### Case 0
            speedTempAcc=0.0
            #speedTempAcc=(-4.4 * (speedTempAcc * speedTempAcc) ) + (26.4 * speedTempAcc) +25
            change_mode("Normal")
            print speed , "move_base zero speed"

        elif speed > 0.05 : ##### Case 2
            if (mode !="Forward"):
                rospy.loginfo("mode wasn't forward ")
                change_mode("Forward")
                mode_time=time.time()
            speedTempAcc=speed
            #speedTempAcc=(-4.4 * (speedTempAcc * speedTempAcc) ) + (26.4 * speedTempAcc) +25
#            print "+ve speed"

        elif speed < -0.05 : ##### Case 3
            if (mode != "Reverse"):
                rospy.loginfo("mode wasn't backward ")
                change_mode("Reverse")
                mode_time=time.time()
            speedTempAcc=-speed
            #speedTempAcc=(-4.4 * (speedTempAcc * speedTempAcc) ) + (26.4 * speedTempAcc) +25
 #           print "-ve speed"
            
        else : ##### Case 4
            print "else .. There is somethig wrong" 
            
 # Addind 2 conditions to increase or decrease speed in case of slopes 

 # waaaaaAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAarnig you should differ forward and reverse              
    if (  ( (abs(np.mean(Vmps_10))-abs(speed_mod) ) > 0.15 ) and (abs(np.mean(Vmps_10))>0.5 )  and ((np.mean(Vmps_10) != 0.0) ): # descending
        print ("descending slope")
        error= abs(np.mean(Vmps_10)) - abs(speed_mod)
        speed_mod=speed_mod - kp * error
        #speedTempAcc=abs(speed) - kp * error
        
    elif (  ( (abs(speed_mod) - abs(np.mean(Vmps_10)) )> 0.15 ) and (abs(np.mean(Vmps_10))<1.5) and ((np.mean(Vmps_10) != 0.0)  ): #ascending
        print("ascending slope")
        error= abs(speed_mod) - abs(np.mean(Vmps_10))
  #      print ("error",error,kp * error) 
        speed_mod=abs(speed_mod) + kp * error   ## # don't use speedTempAcc to prevent redoing error fixing every time
        
    if(time.time()-mode_time>1.0):
	    DACvalue=(-4.4 * (speedTempAcc * speedTempAcc) ) + (26.4 * speedTempAcc) +25
	    if(int(DACvalue)<=50):
	        print "DAC",DACvalue,"mv_bs",speed,"modified",speedTempAcc,"sensor", np.mean(Vmps_10),"pitch",pitch*180.0/math.pi,rospy.Time.now()	    
	        accelerate(int(DACvalue))
	    else:
	        print "DAC>50,mv_bs,modified,sensor,pitch,now", DACvalue ,speed ,speedTempAcc, np.mean(Vmps_10) , pitch*180.0/math.pi ,rospy.Time.now()
          
    old_speed=speed
#####$$$$$##### keyboard callback #####$$$$$##### 


def callback_speed(data): #add odommsg as global
    global mode, a ,lengthBetweenTwoWheels ,wheelBase ,angle_val,speedTempAcc,  speed , v_mps
    global speed_counter, Vmps_10
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
        vL= 0.0
        vR= 0.0
        vL_sensor= 0.0
        vR_sensor= 0.0

    v_mps= (vR + vL)/ 2.0
    v_mps= a*v_mps
#####add this for mean v_mps
    Vmps_10[speed_counter-1]=v_mps
    speed_counter=speed_counter+1
    if speed_counter>10 :
        speed_counter=1
    wheelAngle=angle_val*math.pi/180.0
 #   print angle_val , "angleval", speedTempAcc, "speedTempAcc", speed, "input speed"
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
  #  v_mps=

def callback_angle(data):
    global angle_val
    angle_val=data.drive.steering_angle

def callback_imu(data):
    global pitch
    roll,pitch,yaw=tf.transformations.euler_from_quaternion([data.orientation.x,data.orientation.y,data.orientation.z,data.orientation.w])
    

def listener():
    rospy.Subscriber("/rbcar_robot_control/command", AckermannDriveStamped, callback_move_base,queue_size=1)
    rospy.Subscriber("/pot_angle", AckermannDriveStamped, callback_angle,queue_size=1)
    rospy.Subscriber("/speed_values", Odometry, callback_speed,queue_size=1)
    rospy.Subscriber("/imu_enu_new", Imu, callback_imu,queue_size=1)
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

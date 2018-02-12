#!/usr/bin/env python
import rospy ,math
import time ,tf
import numpy as np
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import RPi.GPIO as GPIO
from std_srvs.srv import Empty

GPIO.setmode(GPIO.BOARD)

pitch=0.0 # global variable @ callback_imu , callback_mover_base
a=1.0 # parameter to increase wheel odometry average speed (to make distance is closer to reality)
## PI GPIO configrations
modePins    = (36,40,38)
accSwitch   = 35
brakeSwitch = 37
accPins     = (33,32,31,29,26,24)
breakPins   = (23,22,21,19,18,16)
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
command_angle=0.0
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
kp= 0.05  # PID control for speed

mode_time=time.time()
old_speed1=0.0
speedTemp1=0.0
speedTemp1Acc=0.0
MIN_MOVING_SPEED = 0.3
MAX_LOW_SPEED_DAC_VALUE = (-4.4 * (MIN_MOVING_SPEED * MIN_MOVING_SPEED) ) + (26.4 * MIN_MOVING_SPEED) +25
appliedSpeed = 0.0

speed_counter=0
#Vmps_20=[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
Vmps_3=[0.0]
mean_Vmps=0.0

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
#    print("accVal",accVal)
    if accVal <= 12 :
        GPIO.output(accSwitch, 1)
    else:
        GPIO.output(accSwitch, 0)

    accVal = list("{0:b}".format(int(accVal)))
    while len(accVal)<6:
        accVal = ['0']+accVal
    accVal = map(int,accVal)
    GPIO.output(accPins,accVal)
#    print accVal,"DAC pins"

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
                 
#rospy.Timer(rospy.Duration(1.0),timeout_fun)     


#####$$$$$##############################$$$$$$#####      
#####$$$$$##### move_base callback #####$$$$$##### 
DACvalue=15
def callback_move_base(data):
    global old_speed ,mode_time,speedTemp,DACvalue
    speed=data.drive.speed
    if (speed != old_speed ):
        if (speed >= -0.01) and (speed <= 0.01)  : ### Case 0
            change_mode("Normal")
            DACvalue=15

        elif speed > 0.01 : ##### Case 2
            if (mode !="Forward"):
                rospy.loginfo("mode wasn't forward ")
                change_mode("Forward")
                mode_time=time.time()
            DACvalue=speed


        elif speed < -0.01 : ##### Case 3

            
            if (mode != "Reverse"):
                rospy.loginfo("mode wasn't backward ")
                change_mode("Reverse")
                mode_time=time.time()
            DACvalue=-speed                
           
        else : ##### Case 4
            print "else .. There is somethig wrong" 
            

    if(int(DACvalue)<=63):   
        accelerate(int(DACvalue))
        print ("accelerating" , DACvalue)
    else:
        print "DAC>55,mv_bs,modified,sensor,pitch,now"
      
    old_speed=speed
#####$$$$$##### keyboard callback #####$$$$$##### 

vms_counter = 0

def callback_speed(data): #add odommsg as global
    global mode, a ,lengthBetweenTwoWheels ,wheelBase ,angle_val,speedTempAcc,  speed , v_mps
    global speed_counter, Vmps_3,vms_counter
    vms_counter = vms_counter + 1
    left=data.twist.twist.linear.x
    right=data.twist.twist.linear.y
    first_sensor=data.pose.pose.position.x
    second_sensor=data.pose.pose.position.y
    
    if mode=="Reverse":
        vL= -left
        vR= -right
        vF_sensor= -first_sensor
        vS_sensor= -second_sensor

    elif mode=="Forward":
        vL= left
        vR= right
        vF_sensor= first_sensor
        vS_sensor= second_sensor

    else: #change to zero
        vL= 0.0
        vR= 0.0
        vF_sensor= 0.0
        vS_sensor= 0.0

    v_mps= (vR + vL)/ 2.0
#    v_mps= min(vF_sensor ,vS_sensor)
    v_mps= a*v_mps
#####add this for mean v_mps
    Vmps_3[speed_counter]=v_mps
    speed_counter=speed_counter+1
    if speed_counter>0 :
        speed_counter=0
    wheelAngle=angle_val*math.pi/180.0
 #   print angle_val , "angleval", speedTempAcc, "speedTempAcc", speed, "input speed"
    vx = v_mps * math.cos(wheelAngle)
    vy = v_mps * math.sin(wheelAngle)
    odomMsg.twist.twist.linear.x = vx
    odomMsg.twist.twist.linear.y = vy 
    odomMsg.header.frame_id = 'odom'
    odomMsg.child_frame_id = 'base_footprint'
    odomMsg.twist.covariance = [0.0001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1000, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1000, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1000, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1000]
    odomMsg.pose.covariance = [0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    odomMsg.header.stamp= rospy.Time.now()
    odom_pub.publish(odomMsg)
    GPIO.output(19, not(GPIO.input(19)))

def callback_angle(data):
    global angle_val
    angle_val=data.drive.steering_angle

def callback_wheel(data):
    GPIO.output(21, not(GPIO.input(21)))
    

def listener():
    rospy.Subscriber("/rbcar_robot_control/command", AckermannDriveStamped, callback_move_base,queue_size=1)
    rospy.Subscriber("/pot_angle", AckermannDriveStamped, callback_angle,queue_size=1)
    rospy.Subscriber("/speed_values", Odometry, callback_speed,queue_size=1)
    rospy.Subscriber("/wheel_odometry_new", Odometry, callback_wheel,queue_size=1)
    rospy.spin()

def translate(value, leftMin, leftMax, rightMin, rightMax):
    leftSpan = leftMax - leftMin
    rightSpan = rightMax - rightMin
    valueScaled = float(value - leftMin) / float(leftSpan)
    return rightMin + (valueScaled * rightSpan)

def clear_costmap():
    rospy.wait_for_service('/move_base/clear_costmaps')
    try:
        clearing = rospy.ServiceProxy('/move_base/clear_costmaps',Empty)
        test = clearing()
    
    except rospy.ServiceException, e:
        print "Service Call Faild: %s"%e

change_mode("Normal")
if __name__ == '__main__':
    listener()

#!/usr/bin/env python
import rospy ,math
import time ,tf, datetime
import numpy as np
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import RPi.GPIO as GPIO
from std_srvs.srv import Empty
import ctypes , mmap , struct , os
fd = os.open('/tmp/mmaptesting', os.O_CREAT|os.O_RDWR) #os.O_CREAT | os.O_TRUNC | os.O_RDWR)

#assert os.write(fd,'\x00' * mmap.PAGESIZE) == mmap.PAGESIZE
GPIO.setmode(GPIO.BOARD)

pitch = 0.0 # global variable @ callback_imu , callback_mover_base
a = 1.0 # parameter to increase wheel odometry average speed (to make distance is closer to reality)
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

jerk = 1
#global variables for timeout

timeout_feedback_flag = 0

speed = 0.0
command_angle = 0.0
#global variables for change mode
changing = 0
started  = 0
m = 25.64
b = 25.64
#global variables for odometry
lengthBetweenTwoWheels = 1.05
wheelBase = 1.65
v_mps=0.0
angle_val = 0.0
old_speed = 0.0
speedTemp = 0.0
speedTempAcc = 0.0

kp = 1.2 # PID control for speed
#ki = 0.05
#kd = 0

#I=0.0

mode_time=time.time()
old_speed1 = 0.0
speedTemp1=0.0
speedTemp1Acc=0.0
appliedSpeed = 0.0

speed_counter=0
Vmps_3=[0.0]*4
mean_Vmps=0.0

rospy.init_node('Control_node', anonymous=True)
mode_time1=time.time()
odom_pub = rospy.Publisher('/wheel_odometry', Odometry, queue_size=10)
odomMsg = Odometry()
last_command_time=rospy.Time.now()

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
    global timeout_command_flag , jerk , timeout_feedback_flag           
    if (timeout_feedback_flag == 0 ):
        timeout_feedback_flag=1
    else:
        print "delayed feedback"
        if jerk == 1:
            print "disabling move base last command"
            accelerate(13)
rospy.Timer(rospy.Duration(1.0),timeout_fun)


#####$$$$$##############################$$$$$$#####
#####$$$$$##### move_base callback #####$$$$$#####
def callback_move_base(data):
    global timeout_command_flag ,speed , Vmps_3 , pitch ,kp , mean_Vmps , command_angle ,angle_val,mode
    global old_speed ,mode_time,speedTemp,speedTempAcc,speed_counter,vms_counter,last_command_time
    global fd
    last_command_time=rospy.Time.now()

#    print ("vms_counter : ",vms_counter)
    vms_counter = 0
    speed = data.drive.speed     # setpoint
    command_angle = data.drive.steering_angle
    assert os.write(fd,'\x00' * mmap.PAGESIZE)==mmap.PAGESIZE
    buf = mmap.mmap(fd,mmap.PAGESIZE, mmap.MAP_SHARED, mmap.PROT_WRITE)
    i = ctypes.c_float.from_buffer(buf)
    i.value = command_angle
    memSpeed = ctypes.c_float.from_buffer(buf,8)
    memSpeed.value = speed
    original_speed = speed
 #feedback
    if (speed >= -0.01) and (speed <= 0.01) and mode != "Normal" : ### Case 0
        speedTempAcc = 0.0
#            print "clearing", clear_costmap()
        change_mode("Normal")
        mode_time = time.time()
    elif speed < 0.0 and mode != "Reverse" :
        change_mode("Reverse")
        print "Reverse >>"
        mode_time = time.time()
    elif speed > 0.0 and mode != "Forward" :
        change_mode("Forward")
        print "Forward >>"
        mode_time = time.time()

    old_speed=speed
#####$$$$$##### keyboard callback #####$$$$$##### 

vms_counter = 0
last_mode="Normal"
lastSpeed = 0.0

def callback_speed_feedback(data): #add odommsg as global
    global mode, last_mode ,a ,lengthBetweenTwoWheels ,wheelBase, speedTempAcc,speed , v_mps, pitch, kp
    global speed_counter, Vmps_3, vms_counter, command_angle, timeout_feedback_flag,last_command_time

    timeout_feedback_flag = 0
    vms_counter = vms_counter + 1
    left=data.twist.twist.linear.x
    right=data.twist.twist.linear.y
    first_sensor=data.twist.twist.angular.x
    second_sensor=data.twist.twist.angular.y

    if mode=="Reverse":
        if last_mode=="Forward":
	    vL=0.0
	    vR=0.0
#            accelerate(12) # apply brakes when changinf fron forward to back
        else:
            vL= -left
            vR= -right
            vF_sensor= -first_sensor
            vS_sensor= -second_sensor
        last_mode="Reverse"

    elif mode=="Forward":
        if last_mode=="Reverse":
            vL=0.0
            vR=0.0

        else:
            vL= left
            vR= right
            vF_sensor= first_sensor
            vS_sensor= second_sensor
        last_mode="Forward"
    else: #change to zero
        vL= 0.0
        vR= 0.0
        vF_sensor= 0.0
        vS_sensor= 0.0

    v_mps= (vR + vL)/ 2.0
#    if (abs(vF_sensor) +abs(vS_sensor))/2.0 < 1.8:
#    v_mps= min(vL ,vR)
#    else:
#        v_mps= max(vF_sensor ,vS_sensor)
    v_mps= a * v_mps
#####add this for mean v_mps
    Vmps_3[speed_counter] = v_mps
    speed_counter = speed_counter+1
    if speed_counter > 3 :
        speed_counter = 0

    # read angle val from file
    buf = mmap.mmap(fd,mmap.PAGESIZE, mmap.MAP_SHARED, mmap.PROT_READ)
    angle_val, = struct.unpack('f', buf[4:8]) # read feedback angle from the file
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
    mean_Vmps=np.mean(Vmps_3)
########### PID Begad ##################
    if pitch > 0.1 :
        kp = 2.3
    else :
        kp = 1.5
    error = abs(speed) - abs(mean_Vmps)
    P = error*kp
#    I = (I + error) * ki
#    D = (abs(mean_Vmps)-abs(lastSpeed)) * kd
    PIDspeed = P + abs(mean_Vmps)
    if pitch > 0.1 and mode == "Forward":
        PIDspeed = PIDspeed + 0.2
    if mode == "Backward" and PIDspeed < 0.6 :
        PIDspeed = PIDspeed + 0.3
    #lastSpeed = mean_Vmps
    needed_angle_change = command_angle - (angle_val * math.pi/180.0)
    wait_parameter = translate(abs(needed_angle_change), 0.0 , 0.45 , 1.0, 0.0)
    PIDspeed = wait_parameter * PIDspeed
    if (PIDspeed > 0.5):
        DACvalue = 4 * math.sqrt(70 * PIDspeed) + 4.5/PIDspeed
#	print "1st case"
    elif (PIDspeed <= 0.5 and PIDspeed >= 0.0):
        DACvalue = 20 * PIDspeed + 25
#	print "2nd case"
    else :
        DACvalue=0.0
        accelerate(13)
        print " **************ERROR***************** -ve DAC Val", error


    print "Speed: ",speed," PID: ",PIDspeed," DAC Val: ",DACvalue,"kp: " , kp , pitch

#PIDoutput:
    if (rospy.Time.now()-last_command_time>=rospy.Duration(1.0)):
        print "delayed command from feedback"
        accelerate(13)

    else:
        if (int(DACvalue) <= 63 and int(DACvalue) > 0):
            accelerate(int(DACvalue))
#        else:
#            accelerate(63)

pitch = 0.0

def call(data):
#    print "bdbn"
    global pitch
    r,pitch,y= tf.transformations.euler_from_quaternion([data.orientation.x,data.orientation.y,data.orientation.z,data.orientation.w])
#    print "callback" , pitch , "--------------------------------------------"

def callback_wheel(data):
    GPIO.output(21, not(GPIO.input(21)))


def listener():
    rospy.Subscriber("/rbcar_robot_control/command", AckermannDriveStamped, callback_move_base,queue_size=2)
    rospy.Subscriber("/imu_enu_new", Imu , call, queue_size=2)
    rospy.Subscriber("/speed_values", Odometry, callback_speed_feedback,queue_size=2)
    rospy.Subscriber("/wheel_odometry_new", Odometry, callback_wheel,queue_size=2)
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
        print "Service Call Faild: %s" %e

change_mode("Normal")
if __name__ == '__main__':
    listener()

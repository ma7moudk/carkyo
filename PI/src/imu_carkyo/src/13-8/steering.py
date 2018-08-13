#!/usr/bin/env python

'''
steering_angle.py:
    Determines the steering angle from potentiometer
'''
import serial, os,time, mmap
import rospy
from ackermann_msgs.msg import AckermannDriveStamped

# openning file to write feedback
f1 = open('/home/pi/catkin_workspace/src/imu_carkyo/files/angle_fb' , 'w')
f1.seek(0)
f1.write(str(0.0))
f1.truncate()
f1.flush()

# Serial init.
command ='udevadm info -e'
p=os.popen(command,"r")
data = p.read()
lines= data.split('\n\n')
default_port='/dev/ttyACM0'
for i in range(len(lines)):
    lines[i].replace("P","\nP")
    if 'Uno' in lines[i] and 'dev/ttyACM' in lines[i]:
        start = lines[i].find('/dev/ttyACM')
        default_port = lines[i][start:start+12]
ard = serial.Serial(default_port,9600)
ard.flushInput()

rospy.init_node('steering_angle_node')
brakes = AckermannDriveStamped

steering_pub = rospy.Publisher('pot_angle', AckermannDriveStamped, queue_size=10)
steering_msg = AckermannDriveStamped()
steering_angle = 0.0
brakesEn = 0
print_val=0.0
speed=0.0

def readSerial(event):
    global speed,steering_angle,brakesEn,print_val
    tx = time.time()
    lastVal = 0.0
    pot_val = "" 
    
    #print (" $$ SPEED : ",speed)
    if speed == 0.0:
        ard.write('e') #e
        print ("<<<<<< Sending E >>>>>>>>")
    else:
        ard.write('s') #s
    # read angle feedback from arduino
    if (ard.inWaiting()>0):
        pot_val = ard.read(ard.inWaiting())
        pot_val = ard.readline()
        pot_val = ard.readline().strip()
          
    
    try:
        lastVal = steering_angle
        steering_angle = float(pot_val)
    #    print (pot_val)
        print_val=steering_angle
        zero_angle = 516.0
        slope =0.056916104137573
        steering_angle =-slope*steering_angle+ zero_angle*slope # +31.9885557741633  #(-0.04*steering_angle)+20
        
        # 
        #print "Pot:",pot_val,", Angle:",steering_angle
    except ValueError:
        steering_angle=lastVal
   # print ("steering_angle ,pot_val" , steering_angle ,pot_val )
    steering_msg.header.stamp=rospy.Time.now()
    steering_msg.drive.speed = print_val
   # print "opening file to write angle"    
    t = time.time()
    f1.seek(0)
    f1.write(str(steering_angle))
    f1.truncate()
    f1.flush()
    x = time.time()-t
    if x > 0.009:
        print "###################time didff is########################  " , x
    steering_msg.drive.steering_angle = steering_angle #(steering_angle+2.59)*17.0/14.0
    steering_msg.drive.steering_angle_velocity = 0.0
    steering_pub.publish(steering_msg)
    print time.time()-tx

starting=1

last_plan_cmd = time.time()
safe = 0
def callback_move_base(data):
    global speed , last_plan_cmd , starting 
    speed = data.drive.speed
    last_plan_cmd=time.time()


last_speed=15.0

def callback_key(data):
    global speed ,last_plan_cmd ,last_speed, last_speed ,starting
    key_speed = data.drive.speed
    if (time.time()-last_plan_cmd < 0.7):
        #ee = 0
        last_speed = key_speed
    elif (time.time()-last_plan_cmd > 0.7 and abs(last_speed-key_speed) > 0.01):
        last_speed = key_speed
        speed = key_speed
    elif (time.time()-last_plan_cmd > 0.7 and time.time()-last_plan_cmd < 1.5 and abs(last_speed-key_speed) < 0.01):
        speed = 0.0

#    if (time.time()-last_plan_cmd > 0.6  and time.time()-last_plan_cmd <3.0 ):
#        speed = 0.0
#        last_speed=data.drive.speed
#        safe=1
        
#    elif (time.time()-last_plan_cmd > 0.6 and abs(speed-last_speed)<0.01):
#        speed = data.drive.speed
#    else
    
def listener():
    rospy.Subscriber("/rbcar_robot_control/command", AckermannDriveStamped, callback_move_base,queue_size=1)
    rospy.Subscriber("/keyboard", AckermannDriveStamped, callback_key,queue_size=1)
    rospy.Timer(rospy.Duration(0.03), readSerial, oneshot=False)
    rospy.spin()
    
if __name__ == '__main__':
    listener()

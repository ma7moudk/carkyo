#!/usr/bin/env python
import rospy , math , time ,datetime
from ackermann_msgs.msg import AckermannDriveStamped
import RPi.GPIO as GPIO
from std_msgs.msg import Int8
import ctypes , mmap , struct , os
import serial

GPIO.setmode(GPIO.BOARD)
angle_val = 0.0	#received angle from pot node
enable = 1	# enable stepper change only when angle_val is between -30 , +30  *0.26  0.26 -14 14

direction = 12
GPIO.setup(10, GPIO.OUT)
GPIO.setup(8, GPIO.OUT)
GPIO.setup(21,GPIO.OUT)
GPIO.setup(direction, GPIO.OUT)
print "starting"
steering_angle = 0.0
enable_angle_feedback = 0
wait_for_feedback = False
seq = 0
wait_for_callback=True
initialize = True
#time.sleep(10)
#fd = os.open('/tmp/mmaptesting', os.O_RDONLY)
fd = os.open('/tmp/mmaptesting' , os.O_RDWR|os.O_CREAT ) # os.O_CREAT | os.O_TRUNC | os.O_RDWR)
#assert os.write(fd,'\x00' * mmap.PAGESIZE)==mmap.PAGESIZE


angle_val = 0.0
steering_angle = 0.0
cmd_angle=0.0

command ='udevadm info -e'
p=os.popen(command,"r")

data = p.read()
lines=data.split('\n\n')
default_port='/dev/ttyACM0'
for i in range(len(lines)):
    lines[i].replace("P","\nP")
    if 'Uno' in lines[i] and 'dev/ttyACM' in lines[i]:   

        start = lines[i].find('/dev/ttyACM')
        default_port = lines[i][start:start+12]


ard = serial.Serial(default_port, 9600)
ard.flushInput()
eFlag = False
oldFlag = True
t=time.time()

while 1:
    global steering_angle, angle_val, eFlag
#    if((time.time()-t)>0.2):
#        GPIO.output(21,not(GPIO.input(21)))
#        t=time.time()
    pot_val = ""

    # Reading from arduino
    #if (ard.inWaiting()>0):
    #pot_val = ard.read(ard.inWaiting())
    ard.flushInput()
    pot_val = ard.readline()
    pot_val = ard.readline().strip()
#        print(pot_val)
    try:
        lastVal = angle_val
        angle_val = float(pot_val)
        angle_val = -0.056916104137573 * angle_val + 31.9885557741633
    except ValueError:
        angle_val = lastVal

    # getting commanded angle from the file
    assert os.write(fd,'\x00' * mmap.PAGESIZE)==mmap.PAGESIZE
    buf = mmap.mmap(fd,mmap.PAGESIZE, mmap.MAP_SHARED, mmap.PROT_READ)
    cmd_angle_mmap, = struct.unpack('f', buf[:4])# get commanded angle
    cmd_angle=cmd_angle_mmap
    brake, = struct.unpack('f', buf[8:12])  # get commanded speed
    buf = mmap.mmap(fd,mmap.PAGESIZE, mmap.MAP_SHARED, mmap.PROT_WRITE)
    s = ctypes.c_float.from_buffer(buf,4)   # write feedback angle
    s.value = angle_val

    if brake == 0.0:
        eFlag = True
    else:
        eFlag = False

    if eFlag == True and oldFlag == False:
        ard.write('e')
    elif eFlag == False and oldFlag == True:
        ard.write('s')

    oldFlag = eFlag
#    if steering_angle != new_i:
#        steering_angle = new_i

    cmd_angle_deg = float(cmd_angle)*180.0/math.pi
    #print "req ang: ",angle_val_deg , "curr ang" , angle_val
    if (cmd_angle_deg > 14.0):
        cmd_angle_deg = 14.0
    elif (cmd_angle_deg < -14.0):
        cmd_angle_deg = -14.0
    angle = cmd_angle_deg - angle_val
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
#    print ("CMD_ang",cmd_angle_deg , ",pot_angle:",angle_val ,",diff:",angle)
    if enable == 1 and abs(angle) >= 0.75:
        for x in range(10): #min(50,steps)):
            GPIO.output(10,1)
            time.sleep(3/1000.0)
            GPIO.output(10,0)
            time.sleep(3/1000.0)
        GPIO.output(8,1)
#        publish_angel_measure_request(1)

#!/usr/bin/env python

# KZOH: Changed TEN_MICRO to HUNDRED_MICRO (and variables)
# KZOH: Changed function names Thread to: TimmedTasks, EncoderHundredMicroTask, etc.
# KZOH: Removed thread and add a call to TimmedTasks at main
# Add the following functions for Stepper:
#	- ResetStepperPos()
#	- stepper_change(i)

# Accepts motion commands and send it to HW
# Receives HW status: Wheel odom from HW and car controller
##ma7moudk23Oct : seprate mode pins out of switches pins .. Switches value depends on DAC value
#####$$$$$##### Includes #####$$$$$##### 
import time 
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BOARD)
###############################################################
###Global values for encoders######
NumberOfInterrputsPorts = 4   # 4 I/Os
HUNDRED_MICRO = 0.0001 # temp = 1 sec (10.0 / 1000.0) / (1000.0)        # units in sec
HunderedMicros = 0
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

stepper_pos=10.0
direction = 12
SubSteps=0
Steps=0
HighStep=True
StepperDirection=0

pin=[13,15,7,11]
GPIO.setup(pin,GPIO.IN,pull_up_down=GPIO.PUD_UP)
GPIO.setup(10,GPIO.OUT)
GPIO.setup(8,GPIO.OUT)
GPIO.setup(direction,GPIO.OUT)
InStepperChange = False
OneMilliesCounter = 0

def ResetStepperPos():
	global stepper_pos
	print "reset"
	stepper_pos=0.0

def GetStepperPos():
	global stepper_pos
	return stepper_pos

def GetRising():
    global Rising
    return (Rising[0], Rising[1],Rising[2],Rising[3])

def GetFalling():
    global Falling
    return (Falling[0], Falling[1],Falling[2],Falling[3])

def GetTransitions():
    global Transitions
    return (Transition[0], Transition[1],Transition[2],Transition[3])

def stepper_change(i):
    global stepper_pos
    global InStepperChange
    global Steps, SubSteps, HighStep, StepperDirection, direction
    print "Sterrper change fun" , InStepperChange, i
    if  i != 0:
        InStepperChange = True
    #    print "Num of steps : ",i
    #    stepper_pos = stepper_pos + i
        if i < 0:
            GPIO.output(8,0)
            GPIO.output(direction,1)
            i = i*-1
            StepperDirection = -1
        elif i > 0:
            GPIO.output(8,0)
            GPIO.output(direction,0)
            StepperDirection = 1
        Steps = i
        HighStep = True
        SubSteps = 40
        print "Line 93.........SubSteps: ", SubSteps, Steps, HighStep

    #    for x in range(0,i*40):
    #        GPIO.output(10,1)
    #        time.sleep(2.0/1000.0)
    #        GPIO.output(10,0)
    #        time.sleep(2.0/1000.0)
    #    GPIO.output(8,1)

def getSubSteps():
    global SubSteps
    return SubSteps
def OneMilliesTasks ():
    global InStepperChange
    global Steps, SubSteps, HighStep, StepperDirection, stepper_pos, OneMilliesCounter
#    print "One"
    SubSteps=getSubSteps()
    if OneMilliesCounter == 1:
#        print "Two", SubSteps , Steps, StepperDirection
        if SubSteps > 0:
            print "Two", SubSteps , Steps, StepperDirection
            if HighStep == True:
                GPIO.output(10,1)
                HighStep = False
#                print "HighStep == True"
            else:
#                print "HighStep == False"
                GPIO.output(10,0)
                HighStep = True
                SubSteps -= 1
                print SubSteps ,"in if"
                if SubSteps == 0:
                    Steps -= 1
                    stepper_pos += StepperDirection
                    if Steps == 0:
                        GPIO.output(8,1)
                        InStepperChange = False
                        print ""
                        print InStepperChange ,"InStepperChange----*******", stepper_pos
                        print ""
                    else:
#                        print "else put steps 40"
                        SubSteps = 40
    OneMilliesCounter += 1
    if OneMilliesCounter == 2:
        OneMilliesCounter = 0

def HundredMicroTask():
    global HunderedMicros, Millies, Secs, Minutes
    HunderedMicros += 1
    if HunderedMicros >=10:
        HunderedMicros=0
        Millies += 1
        OneMilliesTasks()
        if Millies >=1000:
            Millies=0
            Secs += 1
            if Secs >=60:	
                Secs=0
                Minutes += 1

def EncoderHundredMicroTask():
    global NumberOfInterrputsPorts, Prev, consecutivezeros, consecutiveones, threshold, Falling, Rising, Transition ,pin
#    mydata = threading.local()
#    mydata.i = 1
    for i in range(NumberOfInterrputsPorts):
#       if Port(i) == 0:
        if GPIO.input(pin[i]):
            consecutivezeros[i]+=1
            consecutiveones[i]=0
            if Prev[i] == 1 and consecutivezeros[i] >= threshold[i]:
                Prev[i] = 0
                Falling[i]+=1
                Transition[i]+=1
        else:
            consecutivezeros[i]=0
            consecutiveones[i]+=1
            if Prev[i] == 0 and consecutiveones[i] >= threshold[i]:
                Prev[i] = 1
                Rising[i]+=1
                Transition[i]+=1

def TimmedTasks():
    global TimingError, CorrectTiming, WaitCounter, TimeElapsed
    Start = time.clock() #timeit.timeit()
    Next = Start
    Next = Next + HUNDRED_MICRO
    ResetStepperPos()
    stepper_change(1)
    x=GetStepperPos()
    a=getSubSteps()
    print "Timmes task start"

    #Port[1-n].tmpcount=0, .Rising=0, .Falling=0, .Transition=0, .Prev=0
    while True:
        print "While True", SubSteps , Steps, StepperDirection
        Now = time.clock() # timeit.timeit()
        Start = Now
        if (Now >= Next):	# Exceeds the time
            TimingError+=1
        else:
            CorrectTiming+=1
        while (Now < Next):
            WaitCounter +=1
            Now = time.clock() #timeit.timeit()
        HundredMicroTask()
        EncoderHundredMicroTask()
        Next = Next + HUNDRED_MICRO
        End = time.clock()
        TimeElapsed = End - Start

if __name__ == '__main__':
    TimmedTasks()

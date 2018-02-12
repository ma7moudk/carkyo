#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
from ackermann_msgs.msg import AckermannDriveStamped
import math , time

rospy.init_node('listener', anonymous=True)   
joint_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
jointMsg = JointState()

jointMsg.position=[-0.01965720599515386, -0.08772062772967487, 0.0048052884397185025, -0.0884758230917231, -0.0015572011575821776, -0.004119130977083607, -0.08822036555086142, 1.6201125748281546, -0.05011968349544031, -0.00161701189623642]
jointMsg.velocity=[-0.00015741160963784074, 0.0037008086595842686, -7.142418945675286e-05, 0.0009471160466381012, -0.07740893116240002, -0.00013225947754035104, 0.0010923034269268367, -0.004660447125784095, 0.017859258749365874, 0.09423607590512316]
jointMsg.effort=[0.0, 0.0, 0.0, 0.0, -1.023368809997205, 0.0, 0.0, 0.0, 0.0, 50.0]
jointMsg.name=['a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j']

angle_left=0.0
angle_right=0.0
d1 =0.0
d = 1.65 #RBCAR_D_WHEELS_M
zero=False

def callback_joint(data):
    global angle_left , angle_right
    for i in range(0, 10): #len(data.name)-1):
        jointMsg.name[i]=data.name[i]
        jointMsg.position[i]=data.position[i]
        jointMsg.velocity[i]=data.velocity[i]
        jointMsg.effort[i]=data.effort[i]

    jointMsg.position[4]=angle_left
    jointMsg.position[9]=angle_right
    jointMsg.header.stamp= data.header.stamp
    if (zero == True ):
        jointMsg.position[0]=0.0
        jointMsg.position[1]=0.0
        jointMsg.position[2]=0.0
        jointMsg.position[3]=0.0
        jointMsg.position[5]=0.0
        jointMsg.position[6]=0.0
        jointMsg.position[7]=0.0
        jointMsg.position[8]=0.0
        jointMsg.velocity[0]=0.0
        jointMsg.velocity[1]=0.0
        jointMsg.velocity[2]=0.0
        jointMsg.velocity[3]=0.0
        jointMsg.velocity[4]=0.0
        jointMsg.velocity[5]=0.0
        jointMsg.velocity[6]=0.0
        jointMsg.velocity[7]=0.0
        jointMsg.velocity[8]=0.0
        jointMsg.velocity[9]=0.0
        jointMsg.effort[1]=0.0
        jointMsg.effort[1]=0.0
        jointMsg.effort[2]=0.0
        jointMsg.effort[3]=0.0
        jointMsg.effort[4]=0.0
        jointMsg.effort[5]=0.0
        jointMsg.effort[6]=0.0
        jointMsg.effort[7]=0.0
        jointMsg.effort[8]=0.0
        jointMsg.effort[9]=0.0
        joint_pub.publish(jointMsg)
        print "angle_left" ,jointMsg.position[4],"angle_right" ,jointMsg.position[9]
    else:
        joint_pub.publish(jointMsg)
        print jointMsg

def callback_angle(data):
    global angle_left , angle_right ,d , d1
    angle_deg=data.drive.steering_angle
    angle=angle_deg*math.pi/180.0
    if (angle!=0.0) :
        d1 =  d / math.tan(angle)
        angle_left = math.atan2( d , (d1-0.105))
        angle_right= math.atan2( d , (d1+0.105))
        if (angle < 0.0):
            angle_left = angle_left - math.pi
            angle_right =angle_right- math.pi


    else:
        angle_left = 0.0
        angle_right = 0.0



def listener():
    rospy.Subscriber("/rbcar/joint_states", JointState, callback_joint)
    rospy.Subscriber("/pot_angle", AckermannDriveStamped, callback_angle,queue_size=1)
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
    listener()s

#! /usr/bin/env python
import bluetooth, subprocess
import rospy , os , re
import libnmea_navsat_driver.driver
from nav_msgs.msg import Odometry

rospy.init_node('nmea_serial_driver')


frame_id = libnmea_navsat_driver.driver.RosNMEADriver.get_frame_id()
zero_vel_dur=0.0
startTime = rospy.Time.now()

name = "reach"
addr = '58:A8:39:01:50:0B'
port = 1
passkey = "123456"
subprocess.call("kill -9 `pidof bluetooth-agent`",shell=True)
status = subprocess.call("bluetooth-agent " + passkey + " &",shell=True)
s = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
s.connect((addr,port))
line = ''
data = ''

def callback(data): 
    global vx, vy ,zero_vel_dur, startTime
    vx=data.twist.twist.linear.x
    vy=data.twist.twist.linear.y
    if( (vx+vy)/2.0 < 0.1):
        zero_vel_dur = rospy.Time.to_sec(rospy.Time.now() - startTime)
    else:
        startTime = rospy.Time.now()
        zero_vel_dur = 0.0

def loop_fun():
    global zero_vel_dur,line,data
    try:
        driver = libnmea_navsat_driver.driver.RosNMEADriver()
        while not rospy.is_shutdown():            
            x = s.recv(1)
            if x == '\n':
                print line
                data = line.strip()
                line = ''
            else:
                line += x
            try:
                zero_vel_dur=0.0
                driver.add_sentence(zero_vel_dur , data, frame_id)
            except ValueError as e:
                rospy.logwarn("Value error, likely due to missing fields in the NMEA message. Error was: %s. Please report this issue at github.com/ros-drivers/nmea_navsat_driver, including a bag file with the NMEA sentences that caused it." % e)

    except rospy.ROSInterruptException:
        GPS.close() #Close GPS serial port\



if __name__ == '__main__':
    loop_fun()
#    rospy.Subscriber("wheel_odometry_new", Odometry, callback)
    rospy.spin()
    

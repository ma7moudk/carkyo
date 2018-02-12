#!/usr/bin/env python
import ctypes
import mmap
import os
import struct ,time ,rospy
from ackermann_msgs.msg import AckermannDriveStamped

rospy.init_node("sdb_node")
fd = os.open('/tmp/mmaptest', os.O_CREAT | os.O_TRUNC | os.O_RDWR)
def callback_command(data):
    global fd
    steering=data.drive.steering_angle
    speed=data.drive.speed
    assert os.write(fd, '\x00' * mmap.PAGESIZE) == mmap.PAGESIZE
    buf = mmap.mmap(fd, mmap.PAGESIZE, mmap.MAP_SHARED, mmap.PROT_WRITE)

    i = ctypes.c_float.from_buffer(buf)
    s = ctypes.c_float.from_buffer(buf, 4)

    i.value = steering #int(new_i)
    s.value = speed
    print time.time()


def listener():
    rospy.Subscriber("/rbcar_robot_control/command", AckermannDriveStamped, callback_command)
    rospy.spin()

if __name__ == '__main__':
    listener()

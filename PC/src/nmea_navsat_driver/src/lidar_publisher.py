#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan

rospy.init_node('lidar_node', anonymous=True)   
lidar_pub = rospy.Publisher('/lidar2', LaserScan, queue_size=2)
lidarMsg = LaserScan()


def callback_lidar(data):
    lidarMsg = data
    lidarMsg.header.stamp = rospy.Time.now()
    lidar_pub.publish(lidarMsg)

def listener():
    rospy.Subscriber("/lidar0", LaserScan, callback_lidar)
    rospy.spin()

if __name__ == '__main__':
    listener()

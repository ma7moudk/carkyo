from sensor_msgs.msg import LaserScan
from lidar_lite import Lidar_Lite
import rospy , math

rospy.init_node("Lidar_node")
lidar_msg=LaserScan()
lidar_pub= rospy.Publisher('lidar', LaserScan, queue_size=10)

lidar = Lidar_Lite()
connected = lidar.connect(1)
dist_local=[]
dist_global=[]

while True:
    if connected < -1:
      print "Not Connected"
    distance=lidar.getDistance()
    vel=lidar.getVelocity()
    dist_local.append(distance)
    if (len(dist_local) >=10):
        dist_global=dist_local
        dist_local=[]
        
    
    
    print dist_global
    
def callback(event):
    global dist_global
    lidar_msg.angle_min=-1.0*math.pi
    lidar_msg.angle_max=1.0*math.pi
    lidar_msg.angle_increment=1.0*math.pi/180.0 # angle resolution is 1 deg
    
    # so measurement steps= (90+90)*1=180
    lidar_msg.range_min=0.01 # 1 cm
    lidar_msg.range_max=6.0  # 6 m
    
    # if laserscan give 2 scan per second so time of full scan is 0.5 sec , time of one beam =0.5/180=2.778 ms
    lidar_msg.time_icrement=0.002778
    lidar_msg.scan_time=1 
    lidar_msg.ranges=dist_global
    lidar_msg.header.frame_id=''
    lidar_msg.header.stamp=rospy.Time.now()
    lidar_pub.publish(lidar_msg)
    
rospy.Timer(rospy.Duration(0.2), callback, oneshot=False)

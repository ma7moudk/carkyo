#!/usr/bin/env python
import  rospy
from std_srvs.srv import Empty

def clear_costmap_client():
    rospy.wait_for_service('/move_base/clear_costmaps')
    try:
        clearing=rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
        test=clearing()

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


if __name__ == "__main__":
		print "Requesting" ,clear_costmap_client()

#!/usr/bin/env python
import rosnode , rospy
import rostopic
import collections

started=1

def has_duplicates(list_of_values):  
	value_dict = collections.defaultdict(int) 
	for item in list_of_values: 
		item=item[0:10]
		#if '/rostopic_' ==  item:
			#print "ROSTOPIC Found"
		value_dict[item] += 1  

	return any(val > 1 for val in value_dict.itervalues())  

while True:
	c = 'rostopic'
	x= rosnode.get_node_names()	  

	if has_duplicates(x):  
		print "WAAAAAAAARNING .. There is a duplicate node check rosnode list"
	else: 
		x=0


#		print "The list provided does not contain duplicate values."
##################################################################################
#	y=rostopic._rostopic_cmd_list([c, 'find', 'nav_msgs/Odometry'])
	name=[]
	y=rospy.get_published_topics()
	for i in range(0,len(y)):
		name.append(y[i][0])

	if '/rbcar_robot_control/command' not in  name:
		if started:
			print "/rbcar_robot_control/command not published"
			print "/rbcar_robot_control/command not published"
			started = 0
			

	if '/speed_values' not in  name:
		print "/speed_values not published check encoder node"
		started = 1

	if '/wheel_odometry' not in  name:
		print "/wheel_odometry not published check control node"
		started = 1

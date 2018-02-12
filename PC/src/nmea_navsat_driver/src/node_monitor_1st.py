import rosnode
import collections  

def has_duplicates(list_of_values):  
	print list_of_values
	value_dict = collections.defaultdict(int) 
	print value_dict 
	for item in list_of_values: 
	    item=item[0:12] 
	    print item
	    value_dict[item] += 1  
	return any(val > 1 for val in value_dict.itervalues())  

one=1
while one==1:
	one=0

	x= rosnode.get_node_names()	  
	if has_duplicates(x):  
		print "WAAAAAAAARNING .. There is a duplicate node chech rosnode list"
	else:  
		print "The list provided does not contain duplicate values."

#single_print=1
#single_print_steering_mul=1
#	counter_steering=0

#	for i in range (0,len(x)):
#		if 'steering' in x[i] and counter_steering==0 and single_print==1:
#			print "there is a single steering node"
#			single_print=0
#			counter_steering+=1
#		elif 'steering' in x[i] and counter_steering>0 and single_print_steering_mul==1:
#			print "WAAAAAARNING ..there is multible steering nodes"
#			single_print_steering_mul=0


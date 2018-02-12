#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <unistd.h>
#include <vector>
#include "robotnik_msgs/set_mode.h"
#include "robotnik_msgs/get_mode.h"

ros::ServiceClient set_digital_outputs_client_;  


std::string cmd_service_ptz_;

ros::ServiceClient setKinematicMode;

set_digital_outputs_client_ = nh_.serviceClient<robotnik_msgs::set_digital_output>(cmd_service_io_);
	


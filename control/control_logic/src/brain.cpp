#include <ros/ros.h>
#include "control_logic/MotionCommand.h"
#include "control_logic/info.h"
#include <core_sensors/ir.h>




int main(int argc, char ** argv){

	ros::init(argc, argv, "brain");
	ros::NodeHandle n;
	ros::Rate loop_rate(10);

	ros::ServiceServer request_map = n.serviceClient<nav_msgs::OccupancyGrid>("mapping/ProcessedMap");
	ros::Publisher pub_commands = n.advertise<control_logic::commands>("/brain/commands", 1, readIrData);

	while(ros::ok()){
		loop_rate.sleep();
		ros::spinOnce();
	}

	return 0;

}

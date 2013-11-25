#include <ros/ros.h>
#include "control_logic/MotionCommand.h"
#include "control_logic/info.h"
#include "control_logic/commands.h"
#include <core_sensors/ir.h>
#include <nav_msgs/OccupancyGrid.h>
#include <theia_services/mapsrv.h>




int main(int argc, char ** argv){

	ros::init(argc, argv, "brain");
	ros::NodeHandle n;
	ros::Rate loop_rate(10);

	ros::ServiceClient request_map = n.serviceClient<theia_services::mapsrv>("mapping/ProcessedMap");
	ros::Publisher pub_commands = n.advertise<control_logic::commands>("/brain/commands", 1);

	while(ros::ok()){
		loop_rate.sleep();
		ros::spinOnce();
	}

	return 0;

}

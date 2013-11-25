#include <ros/ros.h>
#include "control_logic/MotionCommand.h"
#include "control_logic/info.h"
#include "control_logic/commands.h"
#include <core_sensors/ir.h>
#include <nav_msgs/OccupancyGrid.h>
#include <theia_services/mapsrv.h>
#include <theia_services/brain_wall.h>
#include <theia_services/brain_blind.h>

const int freq=100;



int main(int argc, char ** argv){

	ros::init(argc, argv, "brain");
	ros::NodeHandle n;
	ros::Rate loop_rate(freq);
	std::vector<int>commands(1,0);
	std::vector<double>vals(1,0.0);
	theia_services::mapsrv map_req;
	theia_services::brain_wall wall_req;
	theia_services::brain_blind blind_req;
	int slave=1;

	ros::ServiceClient request_map = n.serviceClient<theia_services::mapsrv>("mapping/ProcessedMap");
	ros::ServiceClient order_wall = n.serviceClient<theia_services::brain_wall>("wall_follower/instructions");
	ros::ServiceClient order_blind = n.serviceClient<theia_services::brain_blind>("blind/instructions");
	
	while(ros::ok()){
		
		switch(slave){
		case 1:
			wall_req.request.active=true;
			blind_req.request.active=false;
			order_wall.call(wall_req);
			order_blind.call(blind_req);
			break;
		case 2:
			wall_req.request.active=false;
			blind_req.request.active=true;
			blind_req.request.commands=commands;
			blind_req.request.vals=vals;
			order_wall.call(wall_req);
			order_blind.call(blind_req);
			break;
		}
		
		loop_rate.sleep();
		ros::spinOnce();
	}

	return 0;

}

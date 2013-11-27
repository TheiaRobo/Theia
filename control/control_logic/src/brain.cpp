#include <ros/ros.h>
#include "control_logic/MotionCommand.h"
#include "control_logic/info.h"
#include <core_sensors/ir.h>
#include <nav_msgs/OccupancyGrid.h>
#include <theia_services/mapsrv.h>
#include <theia_services/brain_wall.h>
#include <theia_services/brain_blind.h>
#include <theia_services/corrected_odo.h>
#include <theia_services/object.h>

const int freq=100;
const double NO_VAL=-1234567890;
double x=NO_VAL;
double y=NO_VAL;
char heading='E';

const int black=100;
const int blue=75;
const int gray=50;
const int white=0;

const double resolution_matrix=0.01; // in meter!
const int x_matrix=10/resolution_matrix; 
const int y_matrix=10/resolution_matrix;

std::vector<signed char>  Raw_Map(x_matrix*y_matrix,blue);

void get_odo(theia_services::corrected_odo::ConstPtr msg){
	
	x=msg->x;
	y=msg->y;
	
}

void get_map(nav_msgs::OccupancyGrid::ConstPtr msg){

	Raw_Map=msg->data;

}

void order_slaves(int slave,theia_services::brain_wall wall_req, theia_services::brain_blind blind_req,ros::ServiceClient order_wall,ros::ServiceClient order_blind,std::vector<int> commands, std::vector<double> vals){
	
	switch(slave){
	case 1:
		wall_req.request.active=true;
		blind_req.request.active=false;
		wall_req.request.heading=heading;
		order_wall.call(wall_req);
		order_blind.call(blind_req);
		break;
	case 2:
		wall_req.request.active=false;
		blind_req.request.active=true;
		blind_req.request.size=commands.size();
		blind_req.request.commands=commands;
		blind_req.request.vals=vals;
		blind_req.request.heading=heading;
		order_wall.call(wall_req);
		order_blind.call(blind_req);
		break;
	}

	
}

void get_info(control_logic::info::ConstPtr msg){
	
	heading = msg->info_heading;
	
}

bool close_object(){

	return false;
}

int main(int argc, char ** argv){

	ros::init(argc, argv, "brain");
	ros::NodeHandle n;
	ros::Rate loop_rate(freq);
	std::vector<int>commands(1,0);
	std::vector<double>vals(1,0.0);
	theia_services::mapsrv map_req;
	theia_services::brain_wall wall_req;
	theia_services::brain_blind blind_req;
	theia_services::object object_msg;

	int slave=1;

	ros::ServiceClient request_map = n.serviceClient<theia_services::mapsrv>("/mapping/ProcessedMap");
	ros::ServiceClient order_wall = n.serviceClient<theia_services::brain_wall>("/wall_follower/instructions");
	ros::ServiceClient order_blind = n.serviceClient<theia_services::brain_blind>("/blind/instructions");
	ros::Subscriber odo_sub = n.subscribe("/mapping/corrected_odo",1,get_odo);
	ros::Subscriber info_sub = n.subscribe("/control_logic/info",1,get_info);
	ros::Publisher object_pub = n.advertise<theia_services::object>("/control_logic/object",1);
	
	while(ros::ok()){
		
		// processing to decide what to do
		
		
		if(close_object()){
			object_msg.object=1;
			object_pub.publish(object_msg);
		}
		order_slaves(slave,wall_req,blind_req,order_wall,order_blind,commands,vals);
		loop_rate.sleep();
		ros::spinOnce();
	}

	return 0;

}

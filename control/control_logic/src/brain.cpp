#include <ros/ros.h>
#include "theia_services/MotionCommand.h"
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
double x_i=NO_VAL;
double y_i=NO_VAL;
char heading='E';

const int black=100;
const int blue=75;
const int gray=50;
const int white=0;


const double resolution_matrix=0.01; // in meter!
const int x_matrix=15/resolution_matrix; 
const int y_matrix=15/resolution_matrix;
const int obj_thres=20/(resolution_matrix*100); // in meter to cells

std::vector<signed char>  Raw_Map(x_matrix*y_matrix,white);


double cell_round(double val){
	double cm_res=resolution_matrix*100;
	
	return round(val/cm_res);
	
}


void get_odo(theia_services::corrected_odo::ConstPtr msg){
	
	x=msg->x;
	y=msg->y;
	
}

void get_map(nav_msgs::OccupancyGrid::ConstPtr msg){
	
	for(int i=0; i<Raw_Map.size();i++){
		Raw_Map[i]=msg->data[i];
	}

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
	default:
		wall_req.request.active=false;
		blind_req.request.active=false;
		wall_req.request.heading=heading;
		order_wall.call(wall_req);
		order_blind.call(blind_req);
		break;
	}

	
}

void get_info(control_logic::info::ConstPtr msg){
	
	heading = msg->info_heading;
	
}

bool close_object(){
	
	int x_cell,y_cell;

	if(x!=NO_VAL){
		x_cell=cell_round(x*100);
		y_cell=cell_round(y*100);
		
		switch(heading){
	
		case 'E':
			for(int i=1; i<=obj_thres; i++){
				if(Raw_Map[x_cell+i+y_cell*y_matrix]!=white && Raw_Map[x_cell+i+y_cell*y_matrix] != gray && Raw_Map[x_cell+i+y_cell*y_matrix] != blue && Raw_Map[x_cell+i+y_cell*y_matrix] != black){
					return true;
				}
			}
			break;
		case 'W':
			for(int i=1; i<=obj_thres; i++){
				if(Raw_Map[x_cell-i+y_cell*y_matrix]!=white && Raw_Map[x_cell-i+y_cell*y_matrix] != gray && Raw_Map[x_cell-i+y_cell*y_matrix] != blue && Raw_Map[x_cell-i+y_cell*y_matrix] != black){
					return true;
				}
			}
			break;
		case 'N':
			for(int i=1; i<=obj_thres; i++){
				if(Raw_Map[x_cell+(y_cell+i)*y_matrix]!=white && Raw_Map[x_cell+(y_cell+i)*y_matrix] != gray && Raw_Map[x_cell+(y_cell+i)*y_matrix] != blue && Raw_Map[x_cell+(y_cell+i)*y_matrix] != black){
					return true;
				}
			}
			break;
		case 'S':
			for(int i=1; i<=obj_thres; i++){
				if(Raw_Map[x_cell+(y_cell-i)*y_matrix]!=white && Raw_Map[x_cell+(y_cell-i)*y_matrix] != gray && Raw_Map[x_cell+(y_cell-i)*y_matrix] != blue && Raw_Map[x_cell+(y_cell-i)*y_matrix] != black){
					return true;
				}
			}
			break;
	
		}
	}
	
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
	ros::Subscriber info_sub = n.subscribe("/logic/info",1,get_info);
	ros::Subscriber raw_sub = n.subscribe("/mapping/occ",1,get_map);
	ros::Publisher object_pub = n.advertise<theia_services::object>("/control_logic/object",1);
	
	while(ros::ok()){
		
		// processing to decide what to do
		
		if(closed_perimeter(init_time)){
			ROS_INFO("Closed a perimeter");
			slave=0;
		}
		
		if(close_object()){
			ROS_INFO("OBJECT TOO CLOSE OMGWTFBBQ");
			object_msg.object=1;
			object_pub.publish(object_msg);
		}
		order_slaves(slave,wall_req,blind_req,order_wall,order_blind,commands,vals);
		loop_rate.sleep();
		ros::spinOnce();
	}

	return 0;

}

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
#include <theia_services/end.h>
#include <path_planner/path_srv.h>

const int freq=100;
const double NO_VAL=-1234567890;
double x=NO_VAL;
double y=NO_VAL;
double x_i=NO_VAL;
double y_i=NO_VAL;
double init_time = 45; //sec
double P=0.15; //meter
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
std::vector<signed char>  Proc_Map(x_matrix*y_matrix,white);

ros::Time orig_time;

bool blind_done = true;
bool phase_2 = false;
int goal = -1;


int cell_round(double val){
	double cm_res=resolution_matrix*100;

	return round(val/cm_res);

}


void get_odo(theia_services::corrected_odo::ConstPtr msg){

	if(x_i==NO_VAL){
		x_i=msg->x;
		y_i=msg->y;
	}

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
		blind_done = true;
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
		blind_done = blind_req.response.done;
		if(blind_done)
			ROS_WARN("Blind is done");
		break;
	default:
		wall_req.request.active=false;
		blind_req.request.active=false;
		wall_req.request.heading=heading;
		order_wall.call(wall_req);
		order_blind.call(blind_req);
		// blind_done = blind_done --> no change in status is intended
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

bool time_for_more(ros::Time orig){
	
	if((ros::Time::now().toSec()-orig.toSec()) > 5*60){
		ROS_WARN("NO TIME FOR MORE!");
		getchar();
		phase_2=true;
		return false;
		
	}
	
	return true;
	
}

bool closed_perimeter(ros::Time init){

	/*if(!time_for_more(orig_time)) // NOT WORKING
		return true;*/
	
	//ROS_INFO("Distance from starting position: %.2f",sqrt((x-x_i)*(x-x_i)+(y-y_i)*(y-y_i)));
	if(x_i!=NO_VAL){

		if((ros::Time::now().toSec()-init.toSec()) > init_time){
			return true;
			if(sqrt((x-x_i)*(x-x_i)+(y-y_i)*(y-y_i) < P))
				return true;

		}

	}

	return false;

}

int goal_picker(){
	
	ROS_INFO("Press any key to go back to the start...");
	getchar();
	
	return -1;
	
}

void reset_odo_mapping(ros::Publisher p){
	theia_services::end msg;
	
	msg.stop=true;
	
	p.publish(msg);
	
	
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
	path_planner::path_srv path_req;

	int slave=1;
	ros::Time init_time = ros::Time::now();
	orig_time = ros::Time::now();
	

	ros::ServiceClient request_map = n.serviceClient<theia_services::mapsrv>("/mapping/ProcessedMap");
	ros::ServiceClient order_wall = n.serviceClient<theia_services::brain_wall>("/wall_follower/instructions");
	ros::ServiceClient order_blind = n.serviceClient<theia_services::brain_blind>("/blind/instructions");
	ros::ServiceClient request_path = n.serviceClient<path_planner::path_srv>("/path_planner/plan_trajectory");

	ros::Subscriber odo_sub = n.subscribe("/mapping/corrected_odo",1,get_odo);
	ros::Subscriber info_sub = n.subscribe("/control_logic/info",1,get_info);
	ros::Subscriber raw_sub = n.subscribe("/mapping/occ",1,get_map);

	ros::Publisher object_pub = n.advertise<theia_services::object>("/control_logic/object",1);
	ros::Publisher phase_2_pub = n.advertise<theia_services::end>("/control_logic/stop",1);

	while(ros::ok()){

		// processing to decide what to do


		if(closed_perimeter(init_time)){
			if(time_for_more(orig_time) && !phase_2){
				if(blind_done){

					ROS_INFO("Closed a perimeter and I have time for more");
					ROS_WARN("I'll call the path planner");
					order_slaves(0,wall_req,blind_req,order_wall,order_blind,commands,vals);
					if(request_map.call(map_req)){

						path_req.request.map = map_req.response.map;
						path_req.request.x = cell_round(x*100);
						path_req.request.y = cell_round(y*100);
						path_req.request.goal = gray; // we want to explore more :)
						path_req.request.heading = heading;

						if(request_path.call(path_req)){
							
							commands.resize(path_req.response.size);
							vals.resize(commands.size());
							for(int i=0; i < commands.size(); i++){
								commands[i] = path_req.response.commands[i];
								vals[i] = path_req.response.vals[i];
							}

							if(path_req.response.size<=1){
								ROS_WARN("No path found. I will go back to the start");

								path_req.request.map = map_req.response.map;
								path_req.request.x = cell_round(x*100);
								path_req.request.y = cell_round(y*100);
								path_req.request.goal = -1; // back to the start
								path_req.request.heading = heading;

								if(request_path.call(path_req)){

									commands.resize(path_req.response.size);
									vals.resize(commands.size());
									for(int i=0; i < commands.size(); i++){
										commands[i] = path_req.response.commands[i];
										vals[i] = path_req.response.vals[i];
									}
								}

								phase_2=true;

							}

							slave = 2;
						}else{

							ROS_ERROR("Could not get path from path_planner");

						}

					}else{

						ROS_ERROR("Could not get processed map from mapping");

					}

				}


			}else{

				if (!phase_2){

					ROS_WARN("Ran out of time :(");
					phase_2 = true;
					reset_odo_mapping(phase_2_pub);
					heading = 'E';
				}

				if(blind_done){
					ROS_INFO("Ready to start phase 2");
					goal = goal_picker();
					ROS_WARN("I'll call the blind");
					order_slaves(0,wall_req,blind_req,order_wall,order_blind,commands,vals);
					if(request_map.call(map_req)){
	
						path_req.request.map = map_req.response.map;
						path_req.request.x = cell_round(x*100);
						path_req.request.y = cell_round(y*100);
						path_req.request.goal = goal; // the chosen object
						path_req.request.heading = heading;
	
						if(request_path.call(path_req)){
	
							commands.resize(path_req.response.size);
							vals.resize(commands.size());
							for(int i=0; i < commands.size(); i++){
								commands[i] = path_req.response.commands[i];
								vals[i] = path_req.response.vals[i];
							}
	
							if(path_req.response.size<=1){
								ROS_WARN("No path found.");
								slave = 0;
							}else{
	
								slave = 2;
							}
						}else{
	
							ROS_ERROR("Could not get path from path_planner");
	
						}
	
					}else{
	
						ROS_ERROR("Could not get processed map from mapping");
	
					}
				}

			}
		}else if(blind_done && slave == 2){
			
			ROS_INFO("Will explore another perimeter");
			getchar();
			x_i=x;
			y_i=y;
			init_time = ros::Time::now();
			
			slave = 1;
			
			blind_done = false;
			
			
		}


		if(close_object()){
			ROS_INFO("OBJECT TOO CLOSE");
			object_msg.object=1;
			object_pub.publish(object_msg);
		}

		order_slaves(slave,wall_req,blind_req,order_wall,order_blind,commands,vals);
		loop_rate.sleep();
		ros::spinOnce();
	}

	return 0;

}


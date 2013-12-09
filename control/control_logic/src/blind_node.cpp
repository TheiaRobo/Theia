
#include <ros/ros.h>
#include "theia_services/MotionCommand.h"
#include "control_logic/info.h"
#include <core_sensors/ir.h>
#include <theia_services/brain_blind.h>
#include <theia_services/blind_done.h>

///////////////////////////////////////////////////////////////////////////////////////////////////////////
// 
// VARIABLE DEFINITIONS
//  
///////////////////////////////////////////////////////////////////////////////////////////////////////////

const float PI = 3.1415926f;

//IR distance thresholds
double front_max = 20.0;
double front_min = 6.0;
double side_max = 20.0;
double side_min = 4.0;
double side_ref = 4.0;
double cross_thres1=0;
double cross_thres2=0;

int last_turn = 0;	//0 - null, 1 - left, 2 - right
int last_direction = 0; //0 - null, 1 - left, 2 - right, 3 - forward
double range_exc = 20.0;
double info_heading_ref = 0;
double drive_mode = 0;
double forward_standard = 22.0;
double forward_medium = 32.0;
double forward_extended = 37.0;
const int hist_size = 100;
const int ir_size = 8;
const int median_size = 3;
int current_idx = 0;

// info stuff
char info_heading='E';
int info_wall=0;

ros::Publisher info_pub;
ros::Publisher done_pub;

std::vector<int> b_commands;
std::vector<double> b_parameters;

bool done=false;
bool active=false;

///////////////////////////////////////////////////////////////////////////////////////////////////////////
// 
// MISC FUNCTIONS
//  
///////////////////////////////////////////////////////////////////////////////////////////////////////////

void publish_info(){
	control_logic::info msg;
    
	msg.info_heading=info_heading;
	msg.info_wall=info_wall;
    
	info_pub.publish(msg);
    
}

void change_heading(char delta_heading){

	switch(info_heading){

	case 'E':
		if(delta_heading=='R')
			info_heading='S';
		else if (delta_heading == 'L')
			info_heading='N';
		else
			info_heading='W';
		break;
	case 'N':
		if(delta_heading=='R')
			info_heading='E';
		else if (delta_heading == 'L')
			info_heading='W';
		else
			info_heading='S';
		break;
	case 'W':
		if(delta_heading=='R')
			info_heading='N';
		else if (delta_heading == 'L')
			info_heading='S';
		else
			info_heading='E';
		break;
	case 'S':
		if(delta_heading=='R')
			info_heading='W';
		else if (delta_heading == 'L')
			info_heading='E';
		else
			info_heading='N';
		break;
	}

}

void warn_brain(){
	theia_services::blind_done msg;
	
	msg.done=done;
	
	done_pub.publish(msg);

}

///////////////////////////////////////////////////////////////////////////////////////////////////////////
// 
// EXECUTE SECTION
//  
///////////////////////////////////////////////////////////////////////////////////////////////////////////

/*
 * think: will update &res after a given &req
 * */
bool execute(theia_services::MotionCommand::Request &req, theia_services::MotionCommand::Response &res){

	if(!active){ // motion will have to ask somewhere else
		res.B=0;
		return true;
	}
	
	if(done){
		//ROS_INFO("IM DONE. PRESS ANY KEY TO GO ON");
		//getchar();
		warn_brain();
		res.B=0;
		return true;
	}
	
	info_wall=0;
	
	ros::Duration refresh(0.1);
	refresh.sleep(); // wait a bit before sending new orders
	
	
	res.B = b_commands[current_idx];
	switch (b_commands[current_idx]){
	case 1: 
		res.parameter = b_parameters[current_idx];
		info_wall = 3;
		break;
	case 2: 
		res.parameter = b_parameters[current_idx];
		info_wall = -1;
		if (b_parameters[current_idx] == PI/2){
			change_heading('L');
		}else if (b_parameters[current_idx] == -PI/2){
			change_heading('R');
		}else if (b_parameters[current_idx] == PI){
			change_heading('F');
		}
		break;
	default:
		ROS_ERROR("Wrong instruction in the list");
		done = true;
		res.B = 0;
		warn_brain();
		return true;
	}
	
	current_idx++;

	if(current_idx >= b_parameters.size()){
		ROS_WARN("Finished the instructions.");
		//getchar();
		done = true;
		if(current_idx==b_parameters.size())
			warn_brain();
		res.B=0;
		return true;
		
	}

	///////////////////////////////////////////////////
	//Finished cases. Debug
	///////////////////////////////////////////////////
	
	switch(res.B){
	case 1:
		ROS_INFO("GO FORWARD %.2f",res.parameter);
		break;
	case 2:
		ROS_INFO("TURN %.2f",res.parameter);
		break;
	case 3:
		ROS_INFO("FOLLOW WALL: %.2f",res.parameter);
		break;
	default:
		ROS_ERROR("WRONG INSTRUCTION");
		break;
	}

	//getchar();

	publish_info();

	//loop_rate.sleep();
	ros::spinOnce();

	//Update in history vector
	return true;
}

bool status(theia_services::brain_blind::Request &req, theia_services::brain_blind::Response &res){

	done = false;
	active=req.active;
	info_heading=req.heading;

	b_parameters.resize(req.size);
	b_commands.resize(req.size);

	for (int i=0; i< req.size; i++){
		b_parameters[i]=req.vals[i];
		b_commands[i]=req.commands[i];
	}

	if(active)
		ROS_INFO("I'm being called and I am active");	
	else
		ROS_INFO("I'm being called and I am not active");

	if(req.size<=1 && active){
		ROS_WARN("There is no path");
		active = false;
		done = true;
		warn_brain();
	}
	if(req.size>1 && active){
		ROS_INFO("GOT A NEW PATH.");
	}
	
	res.done=true;
	
	return true;
}



///////////////////////////////////////////////////////////////////////////////////////////////////////////
// 
// MAIN() SECTION
//  
///////////////////////////////////////////////////////////////////////////////////////////////////////////

int main(int argc, char ** argv){

	ros::init(argc, argv, "blind_node");
	ros::NodeHandle n;
	ros::Rate loop_rate(100);

	ros::ServiceServer blind_service = n.advertiseService("/blind_node/motion_command", execute);	
	ros::ServiceServer orders = n.advertiseService("/blind/instructions",status);

	info_pub = n.advertise<control_logic::info>("/control_logic/info",1);
	done_pub = n.advertise<theia_services::blind_done>("/blind/done",1);

	while(ros::ok()){
		loop_rate.sleep();
		ros::spinOnce();
	}

	return 0;

}

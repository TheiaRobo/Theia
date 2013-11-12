#include <ros/ros.h>
#include "control_logic/MotionCommand.h"
#include <core_sensors/ir.h>

double ir[8] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
double ir_raw[8][3];
const float PI = 3.1415926f;
double fwd_buffer = 10.0;
double sde_buffer = 20.0;
double crs_buffer = 30.0;
int last_turn = 0;	//0 - null, 1 - left, 2 - right
int last_direction = 0; //0 - null, 1 - left, 2 - right, 3 - forward
double range_exc = 20.0;
double heading_ref = 0;
double drive_mode = 0;

double median(double ir[3]){ // Simple 3-value median filter
	double temp, ir_temp[3];

	for(int i=0; i < 3; i++)
		ir_temp[i]=ir[i];

	for(int i=0; i < 3; i++){
		for(int j=i+1; j<3; j++){
			if(ir[i]>ir[j]){
				temp=ir_temp[i];
				ir_temp[i]=ir_temp[j];
				ir_temp[j]=temp;
			}
		}
	}

	return ir_temp[1];

}

/* Function that should update, the IR values obtained from the core, after filtering*/
void readIrData(core_sensors::ir::ConstPtr ir_msg){

	for(int i=0; i<8; i++){
		for(int j=0; j<2;j++){
			ir_raw[i][j+1]=ir_raw[i][j];
		}
	}

	for(int i=0; i<8; i++){
		ir_raw[i][0]=ir_msg->dist[i];
		ir[i]=median(ir_raw[i]);
	}

}


/*
 * 
 * function: think
 * Mainly the brain of the robot. It will decide that to do given certain conditions of ir and wall 
 * 
 * */
bool think(control_logic::MotionCommand::Request &req, control_logic::MotionCommand::Response &res){

	bool turn=true;

	if(ir[0]==0 && ir[1]==0 && ir[2]==0 && ir[3]==0 && ir[4]==0 && ir[5]==0 && ir[6]==0 && ir[7]==0){
		res.B=0;
		return true;
	}

	ros::Duration refresh(0.1);

	refresh.sleep(); // wait a bit before sending new orders



	res.heading_ref = heading_ref;
	res.B = drive_mode;

	return true;
}

int main(int argc, char ** argv){

	ros::init(argc, argv, "control_logic");
	ros::NodeHandle n;
	ros::Rate loop_rate(10);

	ros::ServiceServer motion_command = n.advertiseService("control_logic/motion_command", think); //Set up service server in this node
	ros::Subscriber ir_data = n.subscribe("/core_sensors_ir/ir", 1, readIrData);

	for(int i=0; i<8; i++)
		for(int j=0; j<3; j++)
			ir_raw[i][j]=0;

	while(ros::ok()){
		loop_rate.sleep();
		ros::spinOnce();
	}

	return 0;

}

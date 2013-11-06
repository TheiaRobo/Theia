#include <ros/ros.h>
#include "control_logic/MotionCommand.h"
#include <core_sensors/ir.h>

double ir[8] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
const float PI = 3.1415926f;

void readIrData(const core_sensors::ir::ConstPtr& msg){

  /*  Forward s/r: 0,1
   *  Left s/r: 2,3
   *  Right s/r: 4,5
   *  Angled l/r: 6,7
   */
   
  for(int i=0; i<8; i++){	//copy ir data to global variables
    ir[i] = msg->dist[i];
  }
  
  /*
  for(int i=0; i<8; i++){
    ROS_INFO("IR sensor %d reads %.2f", i, ir[i]);
  }
  */

  
}

bool think(control_logic::MotionCommand::Request &req, control_logic::MotionCommand::Response &res){
 

  /*
  if(req.A == true){
    res.B = true;
    ROS_INFO("service from control_motion recieved");
  } else {
    res.B = false;
  }
  */ 

  if(ir[0]==0 && ir[1]==0 && ir[2]==0 && ir[3]==0 && ir[4]==0 && ir[5]==0 && ir[6]==0 && ir[7]==0){
	res.B=0;
	return true;
  }

  ros::Duration refresh(1);
  
  refresh.sleep(); // wait a bit before sending new orders
	
  res.B=0; //default
  if(ir[0] > 10 && ir[1] > 10){

	if(ir[2] < 15 && ir[3] < 15)
		res.B=3;
	else if(ir[4] < 15 && ir[5] < 15)
		res.B=3;
		
		else
			res.B=1;
  } else if(ir[2] > 10 && ir[3] > 10){
    res.B = 2;
    res.heading_ref = PI/2;
  } else if(ir[4] > 10 && ir[5] > 10){
    res.B = 2;
    res.heading_ref = -PI/2;
  } else if(ir[2] < 10 && ir[3] < 10 && ir[4] < 10 && ir[5] < 10){ // turn back
	ROS_INFO("TURN BACK\nir3: %.2f\nir4: %.2f\n ir5: %.2f\n ir6: %.2f",ir[2],ir[3],ir[4],ir[5]);
	res.B = 2;
	res.heading_ref=PI;
  }

  return true;
}

int main(int argc, char ** argv){
  
  ROS_INFO("Control Logic (The Brain) has started... hold onto your butts");

  ros::init(argc, argv, "control_logic");
  ros::NodeHandle n;
  

  ros::ServiceServer motion_command = n.advertiseService("control_logic/motion_command", think); //Set up service server in this node
  ros::Subscriber ir_data = n.subscribe("/core_sensors_ir/ir", 1, readIrData); 

  ros::spin();

  return 0;

}

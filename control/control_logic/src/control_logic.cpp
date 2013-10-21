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

  if(ir[0] > 20 && ir[1] > 20){
    res.B = 1;
  } else if(ir[2] > 20 && ir[3] > 20){
    res.B = 2;
    res.heading_ref = PI/2;
  } else if(ir[4] > 20 && ir[5] > 20){
    res.B = 2;
    res.heading_ref = -PI/2;
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

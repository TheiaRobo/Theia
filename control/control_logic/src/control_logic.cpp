#include <ros/ros.h>
#include "control_logic/MotionCommand.h"

bool think(control_logic::MotionCommand::Request &req, control_logic::MotionCommand::Response &res){
  
  if(req.A == true){
    res.B == true;
    ROS_INFO("service from control_motion recieved");
  } else {
    res.B == false;
  }

  return true;
}

int main(int argc, char ** argv){
  
  ROS_INFO("Control Logic (The Brain) has started... hold onto your butts");

  ros::init(argc, argv, "control_logic");
  ros::NodeHandle n;

  ros::ServiceServer motion_command = n.advertiseService("control_logic/motion_command", think);

  ros::spin();

  return 0;

}

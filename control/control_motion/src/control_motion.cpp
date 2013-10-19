/** control_motion: Mid level controller responsible for the movement of the robot as a whole.
*
*	Default behaviours: 'Forward', 'Rotate xº', 'Forward with wall', 'None'
*
*	None: Send (v,w)=(0,0) command to the core and asks for instructions to the control_logic node
*
*	Forward: Makes the system move forwards, keeping the current heading. Goes to None when sensor readings indicate nearby obstacle.
*
*	Rotate xº: Rotates x rads on spot. x>0 means a left turn. Goes to None when the heading is with the desired value, with an error tolerance.
*
*	Forward with wall: Goes forward, trying to keep parallel to the closest wall. Goes to none when sensor readings report no walls to the sides. Goes to Forward when one of the lateral sensors on the wall's side detects a significant change in estimated distance (meaning the robot is on the wall's limit, for instance).
**/


#include "ros/ros.h"
#include <cmath>
#include <core_control_motor/vw.h>
#include <nav_msgs/Odometry.h>
#include <core_sensors/ir.h>
#include <control_logic/MotionCommand.h>
#include <tf/transform_datatypes.h>

double freq=10.0;
double x=0.0,y=0.0,theta=0.0; // Position estimate given by the odometry
double ir_readings[8];

/* Function that should update the odometry values obtained from the core*/
void odo_proc(nav_msgs::Odometry::ConstPtr odo_msg){
	tf::Pose pose;
	tf::poseMsgToTF(odo_msg->pose.pose,pose);
	
	x=odo_msg->pose.pose.position.x;
	y=odo_msg->pose.pose.position.y;
	theta=tf::getYaw(pose.getRotation());
	
	//Debug
	ROS_INFO("Got Pose: (x,y,theta)=(%.2f,%.2f,%.2f)",x,y,theta);
	

}

/* Function that should update the IR values obtained from the core*/
void ir_proc(core_sensors::ir::ConstPtr ir_msg){

	for(int i=0; i<8; i++)
		ir_readings[i]=ir_msg->dist[i];
	
	//Debug
	ROS_INFO("Got ir 1: %.2f\nGot ir 2: %.2f",ir_readings[0],ir_readings[1]);

}

int main(int argc, char ** argv){

	ros::init(argc, argv, "control_motion");
	ros::NodeHandle n;
	ros::Subscriber	odo_sub;
	ros::Subscriber ir_sub;
	ros::Publisher vw_pub;
	ros::Rate loop_rate(freq);
	
        ros::ServiceClient brain = n.serviceClient<control_logic::MotionCommand>("control_logic/motion_command");
        control_logic::MotionCommand srv;
        srv.request.A = true;

	vw_pub = n.advertise<core_control_motor::vw>("/control_motion/vw",1);
	odo_sub = n.subscribe("/core_sensors/odometry",1,odo_proc);
	ir_sub = n.subscribe("/core_sensors/ir",1,ir_proc);
	
	ROS_INFO("Started the control_motion node");
	
	// initialize ir_readings vector
	for(int i=0;i<8;i++)
		ir_readings[i]=0.0;
	
	
	
	while(ros::ok()){
		
          /*if(brain.call(srv)){
            ROS_INFO("service successful!!!");
          } else {
            ROS_INFO("service failed");
            return 1;
          }*/
 
		loop_rate.sleep();
		ros::spinOnce();
	}
	





	return 0;

}

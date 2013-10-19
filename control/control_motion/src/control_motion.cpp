/** control_motion: Mid level controller responsible for the movement of the robot as a whole.
*
*	Default behaviors: 'Forward', 'Rotate xº', 'Forward with wall', 'None'
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
double heading_ref=0.0; // reference for the rotate xº behavior

// Control parameters
double k_forward=1.0;
double k_rotate=1.0;
double std_velocity=15.0;
double heading_thres=0.01;
double dist_thres=20.0;

ros::Publisher vw_pub;
ros::ServiceClient ask_logic;

core_control_motor::vw control_message;
control_logic::MotionCommand srv;



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


/** is_close: Checks if any short range ir detects an obstacle
*
*	Will assume obstacle if the ir reading is below a certain threshold
*
**/
int is_close(){

	for(int i=0; i<6; i++){
		if(ir_readings[i]<dist_thres)
			return 1;
	}
	
	return 0;

}

/** none: Implements the 'None' behavior
*
*	Sends a stopping message to the core and asks for instructions to the logic node
*
**/
int none(ros::Rate loop_rate){

	control_message.v=0;
	control_message.w=0;
	
	vw_pub.publish(control_message);

	srv.request.A=true;
	
	
	/* Because a behavior may stay in loop while doing its thing */
	loop_rate.sleep();
	ros::spinOnce();
	
	if(ask_logic.call(srv)){
            ROS_INFO("Got new instruction");
            
            return 0; // Temporary. Will get instruction from control_logic
            
        } else {
            ROS_INFO("Asked for instructions, no answer received");
            return 0;
        }


}

/** forward: Implements the 'Forward' behavior
*
*	Sends a (v>0,w=k*error) message to the core, with the error being based on the odometry
*
**/
int forward(ros::Rate loop_rate){
	
	double initial_theta=theta;
	double heading_error=0.0;
	int obstacle=0;
	
	
	while(ros::ok()){ // Will keep moving forward until sensors report obstacle
		
		obstacle = is_close();
		
		// Debug
		obstacle=0;
		
		if(obstacle==1){
			control_message.v=0;
			control_message.w=0;
			vw_pub.publish(control_message);
			return 0;
		}
		
		
		heading_error=initial_theta-theta;
		
		//Debug
		//ROS_INFO("initial_theta: %.2f\ntheta: %.2f\nheading_error: %.2f",initial_theta,theta,heading_error);
		
		// Some small threshold to account for noise
		if(abs(heading_error)<heading_thres)
			heading_error=0.0;
		
		control_message.v=std_velocity;
		control_message.w=k_forward*heading_error;
		
		vw_pub.publish(control_message);
		
		loop_rate.sleep();
		ros::spinOnce();
	}
		
	

}

/** rotate: Implements the 'Rotate xº' behavior
*
*	Sends a (v=0,w=k*error) message to the core, with the error being based on the odometry
*
**/

int rotate(ros::Rate loop_rate){
	double initial_theta=theta;
	double heading_error=0.0;
	
	while(ros::ok()){
		
		heading_error=initial_theta-theta;
		
		// action completed
		if(abs(heading_error)<heading_thres){
			return 0;
		}
			
		control_message.v=0.0;
		control_message.w=k_rotate*heading_error;
		
		vw_pub.publish(control_message);	
	
		loop_rate.sleep();
		ros::spinOnce();
	
	}
	

}


/** forward_wall: Implements the 'Forward with wall' behavior
*
*	Sends a (v>0, w=k*error) message to the core, with the error being based on the ir readings
*
**/
int forward_wall(ros::Rate loop_rate){
	
	return 0;
}


int main(int argc, char ** argv){

	ros::init(argc, argv, "control_motion");
	ros::NodeHandle n;
	ros::Rate loop_rate(freq);
	
	ros::Subscriber	odo_sub;
	ros::Subscriber ir_sub;	
	
        ask_logic = n.serviceClient<control_logic::MotionCommand>("control_logic/motion_command");
        
        int behaviour=2; //0 - None; 1 - Forward; 2 - Rotate xº; 3 - Forward with wall
        

	vw_pub = n.advertise<core_control_motor::vw>("/control_motion/vw",1);
	odo_sub = n.subscribe("/core_sensors/odometry",1,odo_proc);
	ir_sub = n.subscribe("/core_sensors/ir",1,ir_proc);
	
	ROS_INFO("Started the control_motion node");
	
	//srv.request.A=true;
	
	// initialize ir_readings vector
	for(int i=0;i<8;i++)
		ir_readings[i]=0.0;
	
	
	
	while(ros::ok()){
	
		switch(behaviour){
			case 0: 
				behaviour = none(loop_rate);
				break;
			case 1:
				behaviour = forward(loop_rate);
				break;
			case 2:
				behaviour = rotate(loop_rate);
				break;			
			case 3:
				behaviour = forward_wall(loop_rate);
				break;
		}
 
	}
	





	return 0;

}

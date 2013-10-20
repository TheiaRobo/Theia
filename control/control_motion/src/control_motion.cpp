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
#include <control_motion/params.h>

double freq=10.0;
double x=0.0,y=0.0,theta=0.0; // Position estimate given by the odometry
double ir_readings[8];
double heading_ref=0.0; // reference for the rotate xº behavior

// Control parameters
double k_forward=1.0;
double k_rotate=1.0;

// Forward velocity
double std_velocity=15.0;

// Maximum distance to be travelled while on 'forward' behavior
double forward_distance=25.0;

// Threshold for the sensors
double heading_thres=0.01;
double dist_thres=20.0;

//0 - None; 1 - Forward; 2 - Rotate xº; 3 - Forward with wall
int behavior=0; 


// distance between ir sensors

double ir_dist=20.0;

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

	for(int i=0; i<8; i++){
		ir_readings[i]=ir_msg->dist[i];
	
		//Debug
		ROS_INFO("IR %d: %.2f",i,ir_readings[i]);
	}


}


/** ir_has_changed: Checks if short range status changed
*
*	Given the inital ir_readings, this function monitor each short range ir to check for significant changes. If the the ir is mounted on the side,
*	it checks for changes on both sensors from the same side
*
**/
int ir_has_changed(double * init_readings){

	double jumps[8];
	
	// absolute change in ir_values
	for(int i=0; i<8; i++)
		jumps[i]=std::abs(init_readings[i]-ir_readings[i]);
		
	// I'm assuming pairs (0,1) for left side; (2,3) for right. Check in the robot and correct later		
	
	// Moved away from the wall on the left/right side of the robot
	
	for(int i=0; i<4; i+=2)
		if(init_readings[i]>dist_thres && jumps[i]<dist_thres && init_readings[i+1]<dist_thres && jumps[i+1]>dist_thres)
			return 1;
		
	// Detected wall on sides
	
	for(int i=0; i<4; i+=2)
		if(init_readings[i] > dist_thres && init_readings[i+1] > dist_thres && jumps[i]>dist_thres && jumps[i+1] > dist_thres)
			return 1;
			
	// Detected obstacle in front
	
	for(int i=4; i<6;i++)
		if(init_readings[i]<dist_thres || jumps[i]>dist_thres)
			return 1;
	
	
	
	
	
			
	return 0;
	
}

void update_params(const control_motion::params::ConstPtr msg){
	
	k_forward=msg->k_forward;
	k_rotate=msg->k_rotate;
	std_velocity=msg->std_velocity;
	heading_thres=msg->heading_thres;
	dist_thres=msg->dist_thres;
	forward_distance=msg->forward_distance;
	heading_ref=msg->heading_ref;
	
	ROS_INFO("New params: k_forward: %.2f\nk_rotate: %.2f\nstd_velocity: %.2f\nheading_thres: %.2f\ndist_thres: %.2f\nforward_distance: %.2f",k_forward,k_rotate,std_velocity,heading_thres,dist_thres,forward_distance);
	
	if(msg->behavior!=0){
		behavior=msg->behavior;
		ROS_INFO("Will switch to behavior %d!!",behavior);
	}
	
	
}

/** compute_angle: Computes the angle from two ir readings, assuming they belong to two sensors separated by ir_dist measuring the same obstacle
*
*	Angle's signal indicates if robot is going against or away from the wall
*
**/
double compute_angle(double * ir){
	
	double theta=0.0;

	if(ir[0]>ir[1]){ // going away from wall
		
		theta = atan2(ir[0]-ir[1],ir_dist);
		return theta;	
	}
	
	// going against the wall
	theta = - atan2(ir[1]-ir[0],ir_dist);
	
	return theta;

}

/** stop: Publishes a (v,w)=(0,0) message
*
**/
void stop(){
	
	control_message.v=0;
	control_message.w=0;
	vw_pub.publish(control_message);
	
}

/** none: Implements the 'None' behavior
*
*	Sends a stopping message to the core and asks for instructions to the logic node
*
**/
int none(ros::Rate loop_rate){

	stop();

	srv.request.A=true;
	
	
	/* Because a behavior may stay in loop while doing its thing */
	loop_rate.sleep();
	ros::spinOnce();
	
	if(ask_logic.call(srv)){
            ROS_INFO("Got new instruction");
            
            return behavior; // Temporary. Will get instruction from control_logic
            
        } else {
 		
 	    // to allow change in behavior from external message
            return behavior;
        }


}

/** forward: Implements the 'Forward' behavior
*
*	Sends a (v>0,w=k*error) message to the core, with the error being based on the odometry
*
**/
int forward(ros::Rate loop_rate){
	
	double initial_theta=heading_ref; // needs to receive rotation angle from logic node
	double heading_error=0.0;
	int status_changed=0;
	double initial_ir[8];
	int num_loops=(int) ceil(forward_distance/(std_velocity/freq));
	
	for(int i=0; i<8; i++)
		initial_ir[i]=ir_readings[i];

	ROS_INFO("Debug mode. Behavior is moving forward. Press any key to go on");
	getchar();
	
	for(int i=0; i<num_loops; i++){ // Will keep moving forward until sensors report obstacle or forward_distance is achieved
		
		status_changed = ir_has_changed(initial_ir);
		
		if(status_changed==1){
			stop();
			
			ROS_INFO("Finished the forward behavior due to changing environment!");
			getchar();
			
			return 0;
		}
		
		
		heading_error=initial_theta-theta;
		
		//Debug
		//ROS_INFO("initial_theta: %.2f\ntheta: %.2f\nheading_error: %.2f",initial_theta,theta,heading_error);
		
		// Some small threshold to account for noise
		if(std::abs(heading_error)<heading_thres)
			heading_error=0.0;
		
		control_message.v=std_velocity;
		control_message.w=k_forward*heading_error;
		
		vw_pub.publish(control_message);
		
		loop_rate.sleep();
		ros::spinOnce();
		
	}
		
	ROS_INFO("Finished the forward behavior successfully!");
	getchar();
	
	return 0;

}

/** rotate: Implements the 'Rotate xº' behavior
*
*	Sends a (v=0,w=k*error) message to the core, with the error being based on the odometry
*
**/

int rotate(ros::Rate loop_rate){

	double heading_error=0.0;
	
	
	ROS_INFO("Debug mode. Behavior is rotation on spot. Press any key to go on");
	getchar();
	
	while(ros::ok()){
		
		heading_error=heading_ref-theta;
		
		// action completed
		if(std::abs(heading_error)<heading_thres){
		
			stop();
			
			ROS_INFO("Finished the rotation behavior successfully with error %.2f (absolute value %.2f)!",heading_error, std::abs(heading_error));
			getchar();
		
			return 0;
		}
			
		control_message.v=0.0;
		control_message.w=k_rotate*heading_error;
		
		vw_pub.publish(control_message);	
	
		loop_rate.sleep();
		ros::spinOnce();
	
	}
	
	return 0;
	

}


/** forward_wall: Implements the 'Forward with wall' behavior
*
*	Sends a (v>0, w=k*error) message to the core, with the error being based on the ir readings. Goes back to none when detects a corner or stops seeing the wall
*
**/
int forward_wall(ros::Rate loop_rate){

	double ir_wall[2]={0.0,0.0};
	int wall=0; // 1 - left side; 2 - right side
	
	
	while(ros::ok()){
		
		// check for wall on left side
		if(ir_readings[0] < dist_thres && ir_readings[1] < dist_thres){
			for(int i=0; i<2; i++)
				ir_wall[i]=ir_readings[i];
			
			// keep following previous wall
			if(wall==2){
				wall=2;
			}else{
				ROS_INFO("Following wall to the left!");
				wall=1;
			}
				
		}else{
			if(wall==1){
				stop();
				ROS_INFO("Stopped seeing wall from the left!");
				getchar();
				return 0;
			}
			
			// check for wall on right side
			if(ir_readings[2] < dist_thres && ir_readings[3] < dist_thres){
				for(int i=2; i<4; i++)
					ir_wall[i]=ir_readings[i];
					
				ROS_INFO("Following wall to the right!");
				wall=2;
			} else{
				stop();
				
				ROS_INFO("Could not find wall!");
				getchar();
				return 0;
			}
		}
		
		// check if obstacle ahead
		
		if(ir_readings[4] < dist_thres || ir_readings[5] < dist_thres){
			
			stop();
			
			ROS_INFO("Obstacle ahead!");
			getchar();
			return 0;
		}
		
		
	
	
		loop_rate.sleep();
		ros::spinOnce();
	
	}
		
	return 0;
}


int main(int argc, char ** argv){

	ros::init(argc, argv, "control_motion");
	ros::NodeHandle n;
	ros::Rate loop_rate(freq);
	
	ros::Subscriber	odo_sub;
	ros::Subscriber ir_sub;	
	ros::Subscriber params_sub;
	
        ask_logic = n.serviceClient<control_logic::MotionCommand>("control_logic/motion_command");
        

	vw_pub = n.advertise<core_control_motor::vw>("/control_motion/vw",1);
	odo_sub = n.subscribe("/core_sensors/odometry",1,odo_proc);
	ir_sub = n.subscribe("/core_sensors/ir",1,ir_proc);
	params_sub = n.subscribe("/control_motion/params",1,update_params);
	
	ROS_INFO("Started the control_motion node");
	
	//srv.request.A=true;
	
	// initialize ir_readings vector
	for(int i=0;i<8;i++)
		ir_readings[i]=0.0;
	
	
	
	while(ros::ok()){
	
		switch(behavior){
			case 0: 
				behavior = none(loop_rate);
				break;
			case 1:
				behavior = forward(loop_rate);
				break;
			case 2:
				behavior = rotate(loop_rate);
				break;			
			case 3:
				behavior = forward_wall(loop_rate);
				break;
		}
 
	}
	





	return 0;

}

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

const float PI=3.1415926f;
double freq=10.0;
double x=0.0,y=0.0,theta=0.0,last_theta=0.0;; // Position estimate given by the odometry
double ir_readings[8];
double heading_ref=0.0; // reference for the rotate xº behavior
double theta_correction=0.0;

// Control parameters
double k_forward=1.0;
double k_rotate=1.0;

// Forward velocity
double std_velocity=9.0;

// Maximum distance to be travelled while on 'forward' behavior
double forward_distance=20.0;

// Threshold for the sensors
double heading_thres=0.01;
double dist_thres=5.0;
double inf_thres=15.0;
double rotation_error_thres=0.10;
double delay_thres=4.0; // no real time :(

//0 - None; 1 - Forward; 2 - Rotate xº; 3 - Forward with wall
int behavior=0; 

// Debug
int count=0;

// distance between ir sensors
double ir_dist=20.0;

// Thresholds for the velocities

double V_MAX=50;
double W_MAX=PI/4;

ros::Publisher vw_pub;
ros::ServiceClient ask_logic;

core_control_motor::vw control_message;
control_logic::MotionCommand srv;

ros::Subscriber	odo_sub;
ros::Subscriber ir_sub;
ros::Subscriber params_sub;



/* Function that should update the odometry values obtained from the core*/
void odo_proc(nav_msgs::Odometry::ConstPtr odo_msg){
	tf::Pose pose;
	tf::poseMsgToTF(odo_msg->pose.pose,pose);

	x=odo_msg->pose.pose.position.x;
	y=odo_msg->pose.pose.position.y;
	last_theta=theta;
	theta=tf::getYaw(pose.getRotation());
	
	//Debug
	//ROS_INFO("Got Pose: (x,y,theta)=(%.2f,%.2f,%.2f)",x,y,theta);
	

}

/* Function that should update the IR values obtained from the core*/
void ir_proc(core_sensors::ir::ConstPtr ir_msg){

	for(int i=0; i<8; i++){
		ir_readings[i]=ir_msg->dist[i];
	
		//Debug
		//ROS_INFO("IR %d: %.2f",i,ir_readings[i]);
	}


}


/** ir_has_changed: Checks if short range status changed
*
*	Given the inital ir_readings, this function monitor each short range ir to check for significant changes. If the the ir is mounted on the side,
*	it checks for changes on both sensors from the same side
*
**/
int ir_has_changed(double * init_readings){

	// Detected obstacle in front
	
	for(int i=0; i<2;i++)
		if(ir_readings[i]<dist_thres)
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
	
	ROS_INFO("New params: k_forward: %.2f\nk_rotate: %.2f\nstd_velocity: %.2f\nheading_thres: %.2f\ndist_thres: %.2f\nforward_distance: %.2f\nheading_ref:%.2f",k_forward,k_rotate,std_velocity,heading_thres,dist_thres,forward_distance,heading_ref);
	
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

/** control_pub: sends a custom (v,w) message
*
**/
void control_pub(double v,double w){
	
	double f_v=v,f_w=w;

	if(f_v>V_MAX)
		f_v=V_MAX;
	if(f_v<-V_MAX)
		f_v=-V_MAX;
	
	if(f_w>W_MAX)
		f_w=W_MAX;
	if(f_w<-W_MAX)
		f_w=-W_MAX;
		
	control_message.v=f_v;
	control_message.w=f_w;
	vw_pub.publish(control_message);
}
	
/** discretize: discretizes a double value on increments of step
*
**/

double discretize(double val, double step){

	for(double i=0; i<100/step; i+=step){
		if(std::abs(val-i)<step){
			//ROS_INFO("val %.3f -> i %.3f",val,i);
			return i;
		}
	}
	
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
            ROS_INFO("Got new instruction: %d",srv.response.B);
            
            if(srv.response.B==2){
            	heading_ref=srv.response.heading_ref;
            	ROS_INFO("Rotation command: %.2f",heading_ref);
            }
            
            //getchar();
            
            return srv.response.B; // Temporary. Will get instruction from control_logic
            
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
	double initial_dist=std::sqrt(x*x+y*y),curr_dist=initial_dist;
	
	for(int i=0; i<8; i++)
		initial_ir[i]=ir_readings[i];

	ROS_INFO("Debug mode. Behavior is moving forward.");
	//getchar();
	
	while(std::abs(curr_dist-initial_dist)<forward_distance){ // Will keep moving forward until sensors report obstacle or forward_distance is achieved
		
		status_changed = ir_has_changed(initial_ir);
		ROS_INFO("ir_readings: (%.3f,%.3f)\n",ir_readings[0],ir_readings[1]);

		if(status_changed==1){
			count++; // Quick and dirty solution that avoids premature stoping due to outliers. Should be carried over to the core_sensors_ir node, somehow.
			if(count>=1){
				stop();
				count=0;
				ROS_INFO("Finished the forward behavior due to changing environment!");
			
				//getchar();
				return 0;
			}
		}else{
			count=0;
		}
		
		curr_dist=std::sqrt(x*x+y*y);
		
		heading_error=initial_theta-theta;
		
		//Debug
		//ROS_INFO("initial_theta: %.2f\ntheta: %.2f\nheading_error: %.2f",initial_theta,theta,heading_error);
		
		// Some small threshold to account for noise
		if(std::abs(heading_error)<heading_thres)
			heading_error=0.0;
		
		control_pub(std_velocity,k_forward*heading_error);
		
		loop_rate.sleep();
		ros::spinOnce();
		
	}
	
	stop();
	
	ROS_INFO("Finished the forward behavior successfully!");
	//getchar();
	
	count=0;
	return 0;

}

/** rotate: Implements the 'Rotate xº' behavior
*
*	Sends a (v=0,w=k*error) message to the core, with the error being based on the odometry
*
**/

int rotate(ros::Rate loop_rate){

	double heading_error=0.0;
	double processed_theta=theta+theta_correction; // will have corrections for the pi to -pi jump
	double init_theta=processed_theta;
	double init_error=heading_ref-(processed_theta-init_theta);
	int correction_mode=0, wall=1;
	double ir_wall[2]={0.0,0.0};
	double theta_ref=0.0, theta_meas=0.0, theta_error=0.0;
	
	ROS_INFO("Debug mode. Behavior is rotation on spot. Press any key to go on");
	//getchar();
	
	while(ros::ok()){
		
		if(correction_mode<=0){
			// process theta
		
			if(std::abs(theta-last_theta)>=PI){
				if(theta>last_theta)
					theta_correction+=-2*PI;
				else
					theta_correction+=2*PI;
			}
		
			processed_theta=theta+theta_correction;
		
			heading_error=heading_ref-(processed_theta-init_theta);
		
			ROS_INFO("Theta: %.3f\nCorrected theta: %.3f\nDisplacement: %.3f\nError: %.3f\n",theta,processed_theta,processed_theta-init_theta,heading_error);

			// action completed
			if(std::abs(heading_error)<heading_thres){
		
				stop();
			
				ROS_INFO("Finished the rotation behavior successfully with error %.2f (absolute value %.2f)!",heading_error, std::abs(heading_error));
				//getchar();
				
				if(correction_mode==0)
					correction_mode=1; // to really be sure we can align with walls
				else
					return 0;
			}
			
			
			if(heading_error/init_error<rotation_error_thres && wall==1)
				correction_mode=1;	
			
			
			control_pub(0.0,k_rotate*heading_error);	
	
			loop_rate.sleep();
			ros::spinOnce();
		}else{ // Correction mode: we finished rotating, and now we want to align ourselves with the wall
			
			// check for wall on the left
			if(ir_readings[2] < dist_thres && ir_readings[3] < dist_thres){
				for(int i=2; i<4;i++)
					ir_wall[i-2]=discretize(ir_readings[i],0.1);
					
				wall=1;
			}else{
				if(ir_readings[4] < dist_thres && ir_readings[5] < dist_thres){
					for(int i=4; i<6; i++)
						ir_wall[i-4]=discretize(ir_readings[i],0.1);
						
					wall=2;
				}else{ // in the case that we can align with a wall on the front
					
					if(ir_readings[0] < dist_thres && ir_readings[1] < dist_thres && 0){ //disabled
						for(int i=0; i<1; i++)
							ir_wall[i]=discretize(ir_readings[i],0.1);
					wall=0;
					}else{ //no wall
						wall=0;
						correction_mode=-1; // so that we can exit above
					}
				}
			}
			
			if(wall){
				
				// get angle to wall
	
				theta_meas = compute_angle(ir_wall);
				theta_error = theta_ref - theta_meas;
				
				if(wall==1){
					ROS_INFO("Aligning with left wall");
					theta_error = -theta_error;
				}else{
					ROS_INFO("Aligning with right wall");
				}
		
				ROS_INFO("Theta_error: %.3f\n",theta_error);
				
				if(theta_error<2*heading_thres){
					stop();
					return 0;
				}
				
				
				control_pub(0,k_rotate*theta_error);		
				loop_rate.sleep();
				ros::spinOnce();
			}
		}
			
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
	double theta_ref=0.0, theta_meas=0.0, theta_error=0.0, avg_dist=0.0;
	int wall=0; // 1 - left side; 2 - right side
	
	
	while(ros::ok()){
		
		// check for wall on left side
		if(ir_readings[2] < inf_thres && ir_readings[3] < inf_thres && wall !=2){
			for(int i=2; i<4; i++)
				ir_wall[i-2]=discretize(ir_readings[i],0.1);
			
			//ROS_INFO("Following wall to the left!");
			wall=1;
				
		}else{
			if(wall==1){
				stop();
				ROS_INFO("Stopped seeing wall from the left!");
				//getchar();
				return 1; // Will try to get 'on the open'
			}
		}
			
		// check for wall on right side
		if(ir_readings[4] < inf_thres && ir_readings[5] < inf_thres && wall!=1){
			for(int i=4; i<6; i++)
				ir_wall[i-4]=discretize(ir_readings[i],0.1);
			
			//ROS_INFO("Following wall to the right!");
			wall=2;
		} else{
		
			if(wall==2){
				stop();
		
				ROS_INFO("Stopped seeing wall from the right!");
				//getchar();
				return 1;
			}
		}
		
		
		// check if obstacle ahead
		
		if(ir_readings[0] < dist_thres+delay_thres || ir_readings[1] < dist_thres+delay_thres){
			count++;
			if(count>=1){	
				stop();
				count=0;
				ROS_INFO("Obstacle ahead!");
				//getchar();
				loop_rate.sleep();
				return 0;
			}
		}else
			count=0;
			
		if(!wall){
			loop_rate.sleep();
			return 0;
		}
		
		// get angle to wall
		
		
		theta_meas = compute_angle(ir_wall);
		theta_error = theta_ref - theta_meas;
		
		// for the same angle measurements we want to rotate to the opposite side than that of the right wall
		if(wall==1)
			theta_error=-theta_error;
		
		//ROS_INFO("Theta_error: %.3f\n",theta_error);
		
		avg_dist=(ir_wall[0]);
		
		if(1){//avg_dist > dist_thres && avg_dist < inf_thres){ VERY BUGGY
			ROS_INFO("Normal wall following");
			if(std::abs(theta_error<0.26)){ //15 degrees error
				control_pub(std_velocity,k_rotate*theta_error);		
				loop_rate.sleep();
				ros::spinOnce();
			}else{ //oops
				loop_rate.sleep();
				return 1;
			}
				
		}else if(avg_dist < dist_thres){ // move away from the wall without moving too much
			
			ROS_INFO("Too close to the wall");
			if(wall==1)
				theta_ref=-PI/20;
			if(wall==2)
				theta_ref=PI/20;
				
			theta_error = theta_ref - theta_meas;
			
			control_pub(std_velocity/2,k_rotate*theta_error);
		}
	
	}
	loop_rate.sleep();
	return 0;
}


int main(int argc, char ** argv){

	ros::init(argc, argv, "control_motion");
	ros::NodeHandle n;
	ros::Rate loop_rate(freq);
	
	
	
        ask_logic = n.serviceClient<control_logic::MotionCommand>("control_logic/motion_command");
        

	vw_pub = n.advertise<core_control_motor::vw>("/control_motion/vw",1);
	odo_sub = n.subscribe("/core_sensors_odometry/odometry",1,odo_proc);
	ir_sub = n.subscribe("/core_sensors_ir/ir",1,ir_proc);
	params_sub = n.subscribe("/control_motion/params",1,update_params);
	
	ROS_INFO("Started the control_motion node");
	
	
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

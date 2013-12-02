#include "ros/ros.h"
#include <cmath>
#include <core_control_motor/vw.h>
#include <nav_msgs/Odometry.h>
#include <core_sensors/ir.h>
#include <theia_services/MotionCommand.h>
#include <tf/transform_datatypes.h>
#include <control_motion/params.h>
#include <theia_services/stop.h>

///////////////////////////////////////////////////////////////////////////////////////////////////////////
// 
// VARIABLES DEFINITION
//  
///////////////////////////////////////////////////////////////////////////////////////////////////////////

/** control_motion: Mid level controller responsible for the movement of the robot as a whole.
 *
 *	Default behaviors: 'Forward', 'Rotate xº', 'Forward with wall', 'None'
 *
 *	Sensors:
 *	ir FRONT: ir_readings[0],ir_readings[1]);
 *	ir LEFT : ir_readings[2],ir_readings[3]);
 *	ir RIGHT: ir_readings[4],ir_readings[5]);
 *	ir NONE:  ir_readings[6],ir_readings[7]);
 *	
None: Send (v,w)=(0,0) command to the core and asks for instructions to the control_logic node
 *
 *	Forward: Makes the system move forwards, keeping the current heading. Goes to None when sensor readings indicate nearby obstacle.
 *
 *	Rotate xº: Rotates x rads on spot. x>0 means a left turn. Goes to None when the heading is with the desired value, with an error tolerance.
 *
 *	Forward with wall: Goes forward, trying to keep parallel to the closest wall. Goes to none when sensor readings report no walls to the sides. Goes to Forward when one of the lateral sensors on the wall's side detects a significant change in estimated distance (meaning the robot is on the wall's limit, for instance).
 **/

const float PI=3.1415926f;
double freq=100.0;
double x=0.0,y=0.0,theta=0.0,last_theta=0.0,processed_theta=0.0; // Position estimate given by the odometry
double ir_readings[8];
double ir_raw[8][3];
double heading_ref=0.0; // ref for the rotate xº behavior
double theta_correction=0.0;
double initial_flag_dist2break_1 = 0.0;
double initial_flag_dist2break_3 = 0.0;

int flag_dist2break_1 = 1;
int flag_dist2break_3 = 1;

// Control parameters
double k_forward=1.0;
double k_rotate=1.0;
double i_rotate=0.0;
double d_rotate=0.00;
double cte_rotate=0.14;
double k_align=0.0;//1.5;
double i_align=0.0;
double d_align=0.0;//0.04;
double k_dist=0.02;
double i_dist=0.0;
double d_dist=0.0;
double k_paralel=1.5;
double i_paralel=0.0;
double d_paralel=0.015;


// Forward velocity
double std_velocity=20.0;
double velocity_fw=std_velocity;
double dist_wall_min=0.0;
double epsilon_theta=0.00; // 5 degrees
double epsilon_dist=0.00;
double last_angle = 0.0;

// Maximum distance to be travelled while on 'forward' behavior
double forward_distance=20.0;

// Threshold for the sensors
double heading_thres=0.005;
double align_thres=100;//0.003;
double dist_thres=6.0;
double cross_thres1=0.5;
double cross_thres2=0.5;
double max_angle = 0.723; // atan(max_ir/17);
double ERROR_VAL = -123456789;

double dist_ref=2.0;
double inf_thres=20.0;
double rotation_error_thres=0.10;
double delay_thres=3.0; // no real time :(

//0 - None; 1 - Forward; 2 - Rotate xº; 3 - Forward with wall
int behavior=0; 
int wall_to_follow=0;
int stop_flag=0;

// Debug
int count=0;

// distance between ir sensors
double ir_dist=20.0;

// PID AUX

const double PID_INIT=123456789.0;

// Thresholds for the velocities

double V_MAX=50;
double W_MAX=8*PI;

ros::Publisher vw_pub;
ros::ServiceClient ask_logic;
ros::ServiceClient ask_blind;

core_control_motor::vw control_message;
theia_services::MotionCommand srv;

ros::Subscriber	odo_sub;
ros::Subscriber ir_sub;
ros::Subscriber params_sub;
ros::Subscriber stop_sub;

///////////////////////////////////////////////////////////////////////////////////////////////////////////
// 
// MISC FUNCTION DEFINITION
//  
///////////////////////////////////////////////////////////////////////////////////////////////////////////


/** 
 *	correct_theta: Adds an offset to the heading estimate to make for discontinuities. t -> theta, l_t -> last_theta
 **/
double correct_theta(double t, double l_t){

	if(std::abs(t-l_t)>=PI){
		if(t>l_t)
			theta_correction+=-2*PI;
		else
			theta_correction+=2*PI;
	}

	processed_theta=theta+theta_correction;

	return processed_theta;
}

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

/* Function that should update the odometry values obtained from the core*/
void odo_proc(nav_msgs::Odometry::ConstPtr odo_msg){
	tf::Pose pose;
	tf::poseMsgToTF(odo_msg->pose.pose,pose);

	x=odo_msg->pose.pose.position.x*100;
	y=odo_msg->pose.pose.position.y*100;
	last_theta=theta;
	theta=tf::getYaw(pose.getRotation());
	processed_theta=correct_theta(theta,last_theta);

	//Debug
	////ROS_INFO("Got Pose: (x,y,theta)=(%.2f,%.2f,%.2f)",x,y,theta);

}

/* Function that should update, the IR values obtained from the core, after filtering*/
void ir_proc(core_sensors::ir::ConstPtr ir_msg){

	for(int i=0; i<8; i++){
		for(int j=0; j<2;j++){
			ir_raw[i][j+1]=ir_raw[i][j];
		}
	}

	for(int i=0; i<8; i++){
		ir_raw[i][0]=ir_msg->dist[i];
		ir_readings[i]=median(ir_raw[i]);
	}

	////ROS_INFO("\nIR_READINGS: (%.2f,%.2f)\n",ir_readings[0],ir_readings[1]);

}

void update_params(const control_motion::params::ConstPtr msg){

	/*k_forward=msg->k_forward;
	k_rotate=msg->k_rotate;
	i_rotate=msg->i_rotate;
	d_rotate=msg->d_rotate;
	cte_rotate=msg->cte_rotate;
	k_align=msg->k_align;
	i_align=msg->i_align;
	d_align=msg->d_align;
	k_dist=msg->k_dist;
	i_dist=msg->i_dist;
	d_dist=msg->d_dist;
	k_paralel=msg->k_paralel;
	i_paralel=msg->i_paralel;
	d_paralel=msg->d_paralel;*/
	//std_velocity=msg->std_velocity;
	//heading_thres=msg->heading_thres;
	//dist_thres=msg->dist_thres;
	//forward_distance=msg->forward_distance;
	heading_ref=msg->heading_ref;
	wall_to_follow=msg->wall_to_follow;
	max_angle=msg->max_angle;

	////ROS_INFO("New params: k_forward: %.2f\nk_rotate: %.2f\nstd_velocity: %.2f\nheading_thres: %.2f\ndist_thres: %.2f\nforward_distance:%.2f\nheading_ref:%.2f",k_forward,k_rotate,std_velocity,heading_thres,dist_thres,forward_distance,heading_ref);

	if(msg->behavior!=0){
		behavior=msg->behavior;
		//ROS_INFO("Will switch to behavior %d!!",behavior);
	}

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

	if(f_w>W_MAX){
		ROS_INFO("WMAX");
		f_w=W_MAX;
	}
	if(f_w<-W_MAX){
		ROS_INFO("-WMAX");
		f_w=-W_MAX;
	}

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
			return i;
		}
	}

}

/** wall_in_range: Checks for a nearby wall for a given threshold, thres.
 *
 *	Side: 1 - Left, 2 - Right, 3 - front
 *
 **/
int wall_in_range(int side, double thres, double ir[8]){ 

	switch(side){
	case 1:
		if((ir[2] < thres) && (ir[3] < thres)){
			return 1;
		}else{
			return 0;
		}
		break;
	case 2:
		if((ir[4] < thres) && (ir[5] < thres)){
			return 1;
		}else{
			return 0;
		}
		break;
	case 3:
		if((ir[0] < thres) || (ir[1] < thres)){
			return 1;
		}else{
			return 0;
		}
		break;
	case 4:
		if((ir[6] < cross_thres1 || ir[7] < cross_thres2) && ir[0] > inf_thres && ir[1] > inf_thres){
			ROS_INFO("Crossed!");
			return 1;
		}else{
			return 0;
		}
	default:
		return 1;
	}

}

/** dist_wall: Given the wall returns the closest one of both walls 
 *
 * wall = 1 (left); wall = 2 (right); wall = 12 (both);	
 *
 *	//Not implemented: wall_closest.irvalue=(closest_wall_left+closest_wall_right)/2;
 *
 **/
double dist_wall(int wall_n){

	double dist_wall_left = 0.0;
	double dist_wall_right = 0.0;
	double irvalue = 0.0;

	if(ir_readings[2] < ir_readings[3]) 		// check for closest wall on left side
		dist_wall_left=ir_readings[2];
	else
		dist_wall_left=ir_readings[3];

	if(ir_readings[4] < ir_readings[5]) 		// check for closest wall on right side
		dist_wall_right=ir_readings[4];
	else
		dist_wall_right=ir_readings[5];

	switch (wall_n){

	case 1:
		if(ir_readings[2] < inf_thres && ir_readings[3] < inf_thres)
			return dist_wall_left;
		else
			return -1;
		break;
	case 2:
		if(ir_readings[4] < inf_thres && ir_readings[5] < inf_thres)
			return dist_wall_right;
		else
			return -1;
		break;
	default:
		return -1;
		break;
	}

}

/** compute_angle: Computes the angle from two ir readings, assuming they belong to two sensors separated by ir_dist measuring the same obstacle
 *
 *	Angle signal indicates if robot is going against or away from the wall
 *
 **/
double compute_angle(double *ir){ //ir[2]

	double theta=0.0;

	if(ir[0]>ir[1]){ 	// going away from wall

		theta = atan2(ir[0]-ir[1],ir_dist);
		//ROS_INFO("Theta: %.2f",theta);
		return theta;	
	}else{ 			// going against the wall
		theta = - atan2(ir[1]-ir[0],ir_dist); //The sign of theta = (-) atan2(ir[1]-ir[0],ir_dist) is WRONG?
		//ROS_INFO("Theta: %.2f",theta);
		return theta;
	}

	//ROS_INFO("Oops");
	return 0;
}

/** compute_ir_error: Computes angle error given ir readings
 * 
 *
 **/
double compute_ir_error(int wall, double * ir_wall, double theta_ref){

	double theta_meas,error_theta;
	// get angle to wall
	theta_meas = compute_angle(ir_wall);
	error_theta = theta_ref - theta_meas;

	//ROS_INFO("ANGLE TO WALL: %.2f\nError: %.2f",theta_meas,error_theta);

	if(wall == 1)
		return -error_theta;
	else if(wall == 2)
		return error_theta;
	else
		return 0;
}

/**
 * compute_ir_dist: Computes distance error given ir readings
 **/
double compute_ir_dist(int wall, double ir_wall[2], double dist_ref){

	double dist_meas,error_dist;

	error_dist=dist_ref-(ir_wall[0]+ir_wall[1])/2;

	return error_dist;
}



/** 
 *	sign: Compute the sign of val
 **/
int sign(double val){

	if(val < 0)
		return -1;
	else
		return 1;

	return 1;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////
// 
// CONTROLLER
//  
///////////////////////////////////////////////////////////////////////////////////////////////////////////

/** 
 *	PID_control: Compute the PID control signal given their respective P, I, D gains.
 **/
double PID_control(double P,double I,double D,double * integrator_sum, double * previous_error,double Ref,double val){

	double error=Ref-val;  		
	double P_part=0,I_part=0,D_part=0, total=0;

	//integrator_sum  +=(error*dt);	// I Part
	(*integrator_sum)+=error/freq;

	if(*previous_error==PID_INIT){ // initialization
		*previous_error=error;
	}
	//differentiator_val =(error-previous error)/dt; // D Part
	D_part=(error-(*previous_error))*freq;
	*previous_error=error;

	P_part=P*error;
	I_part=I*(*integrator_sum);
	D_part=D*D_part;

	total=P_part+I_part+D_part;

	return total;
}



/* PARALEL CONTROLLER 
 * Controller that tries to keep a distance and an angle towards a wall
 */		
double paralel_controller(int wall,double ir_wall[2],double t_ref,double d_ref,double * last_E_r, double * I_sum_r, double * last_R_d, double * I_sum_d){

	double error_theta=0.0, error_dist=0.0,u_theta=0.0,u_dist=0.0;
	// get angle and distance to wall
	error_theta=compute_ir_error(wall,ir_wall,t_ref);
	error_dist=compute_ir_dist(wall,ir_wall,d_ref);
	
	if(std::abs(error_theta)>max_angle){
		
		return ERROR_VAL;
		
	}
	
	
	//Control the angle to the wall
	if(std::abs(error_theta) < epsilon_theta || error_dist > 0 ){ //Small epsilon OR wall too close

		u_theta=PID_control(k_paralel,i_paralel,d_paralel,I_sum_r,last_E_r,error_theta,0)/4;
		//control_pub(velocity_fw,u_theta+u_dist);
	}else{
		u_theta=PID_control(k_paralel,i_paralel,d_paralel,I_sum_r,last_E_r,error_theta,0);
		//control_pub(velocity_fw,u_theta+u_dist);
	}

	//Control the distance to the wall
	if(std::abs(error_dist) < epsilon_dist){ //Small epsilon
		u_dist=0;
		//control_pub(velocity_fw,u_theta+u_dist);
	}else{
		if (wall==1)
			u_dist=-PID_control(k_dist,i_dist,d_dist,I_sum_d,last_R_d,error_dist,0);
		else
			u_dist=PID_control(k_dist,i_dist,d_dist,I_sum_d,last_R_d,error_dist,0);

		//control_pub(velocity_fw,u_theta+u_dist);
	}

	if(error_dist > 0){ // too close to wall
		u_dist=5*u_dist;
	}

	return u_theta+u_dist;		
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////
// 
// STATES DEFINITION
//  
///////////////////////////////////////////////////////////////////////////////////////////////////////////

/** STATE 0 
 * 	none: Implements the 'None' behavior
 *	Sends a stopping message to the core and asks for instructions to the logic node
 **/
int none(ros::Rate loop_rate){

	int wall=0, done=0;
	double ir_wall[2]={0.0,0.0},error_theta=0.0,theta_ref=0.0,u_theta=0.0;
	double I_sum=0.0, last_E=PID_INIT;

	stop();

	heading_ref=0.0;

	//ROS_INFO("None: Align mode\nLast Angle: %.2f",last_angle);
	/* Because a behavior may stay in loop while doing its thing */
	loop_rate.sleep();
	ros::spinOnce();

	/*
	 *	Correction mode: we finished rotating, and now we want to align ourselves with the wall
	 */
	while(ros::ok() && !done){	

		if((wall_in_range(1,inf_thres,ir_readings)) && (wall_in_range(2,inf_thres,ir_readings))){		
			if (dist_wall(2) < dist_wall(1)){
				// Wall on the right is closer. Use this one to align
				wall=2;
				for(int i=0; i<2; i++)
					ir_wall[i]=ir_readings[i+4];
			}else{
				// Wall on the left is closer. Use this one to align
				wall=1;
				for(int i=0; i<2; i++)
					ir_wall[i]=ir_readings[i+2];
			}
		}
		else if ((!wall_in_range(1,inf_thres,ir_readings)) && (wall_in_range(2,inf_thres,ir_readings))){
			// Wall on the right
			wall=2;
			for(int i=0; i<2; i++)
				ir_wall[i]=ir_readings[i+4];
		}
		else if ((wall_in_range(1,inf_thres,ir_readings)) && (!wall_in_range(2,inf_thres,ir_readings))){
			// Wall on the left
			wall=1;
			for(int i=0; i<2; i++)
				ir_wall[i]=ir_readings[i+2];
		}
		else if ((!wall_in_range(1,inf_thres,ir_readings)) && (!wall_in_range(2,inf_thres,ir_readings))){
			wall=0;
		}


		if (wall == 1 || wall == 2){
			// get angle to wall
			error_theta=compute_ir_error(wall,ir_wall,theta_ref);
			//ROS_INFO("error_theta: %.3f\n",error_theta);

			if(std::abs(error_theta)<align_thres){
				stop();
				done=1;
			}
			u_theta=PID_control(k_align,i_align,d_align,&I_sum, &last_E,error_theta,0);
			control_pub(0,u_theta);




		}else{ 
			// if (wall == 0) which is equivalent to if ((wall != 1 && wall != 2))
			if (last_angle != 0){
				//heading_ref=-last_angle;
				heading_ref=0;
				stop();
				loop_rate.sleep();
				ros::spinOnce();
				return 2;				
			}
			else{	
				// if (last_angle == 0): There is nothing to align
				stop();
				done=1;
			}
		}
		loop_rate.sleep();
		ros::spinOnce();
	}

	last_angle = 0;
	//Out of the while. Finished align mode
	//ROS_INFO("None: finished align mode");

	/*
	 *	Ask for directions
	 */
	srv.request.A=true;
	if(ask_logic.call(srv)){

		if(srv.response.B!=0){
			if(srv.response.B==2){
				heading_ref=srv.response.parameter;
			}else if(srv.response.B==3){
				wall_to_follow=(int) srv.response.parameter;
			}else{ // forward
				forward_distance=srv.response.parameter;
			}

			return srv.response.B;
		}else if(ask_blind.call(srv)){

			if(srv.response.B==2){
				heading_ref=srv.response.parameter;
			}else if(srv.response.B==3){
				wall_to_follow=(int) srv.response.parameter;
			}else{ // forward
				forward_distance=srv.response.parameter;
			}

			return srv.response.B;

		}

	} else {
		stop();
		// to allow change in behavior from external message
		return behavior;
	}

}

/** STATE 1 
 * 	forward: Implements the 'Forward' behavior
 *	Sends a (v>0,w=k*error) message to the core, with the error being based on the odometry
 **/
int forward(ros::Rate loop_rate){

	double initial_theta=heading_ref; 
	double heading_error=0.0;
	int status_changed=0,wall=0,avoid_flag=0,far_away_flag=1;
	double initial_ir[8], close_ir=0.0;
	double i_x=x, i_y=y;
	double curr_dist=std::sqrt((x-i_x)*(x-i_x)+(y-i_y)*(y-i_y));
	double BreakingRatio_1;
	double last_E_r=PID_INIT,I_sum_r=0.0,last_R_d=PID_INIT,I_sum_d=0.0;
	double ir_wall[2]={0.0,0.0},u_theta=0.0,theta_ref=0.0,temp_k=k_dist,temp_s=std_velocity;

	for(int i=0; i<8; i++)
		initial_ir[i]=ir_readings[i];
	ROS_INFO("Will go forward %.2f cm!",forward_distance);
	// Will keep moving forward until sensors report obstacle or forward_distance is achieved

	k_dist=0.0;
	while(curr_dist<forward_distance){ 

		if(stop_flag){
			stop();
			stop_flag=0;
			ROS_INFO("Object ahead!\n");
			return 0;
		}

		std_velocity=10.0; // sorry :(

		//Distance to wall < delay_thres ---> very close! STOP
		if(wall_in_range(3,dist_thres,ir_readings) || wall_in_range(4,cross_thres1,ir_readings)){ //
			stop();
			loop_rate.sleep();
			
			if(wall_in_range(4,cross_thres1,ir_readings)){ // will go back
			
				std_velocity=-10.0;
				forward_distance=5.0;
				curr_dist = 0;
				i_x=x;
				i_y=y;
				while(curr_dist < forward_distance){
					
					control_pub(std_velocity,u_theta);
				
					curr_dist=std::sqrt((x-i_x)*(x-i_x)+(y-i_y)*(y-i_y));
					ros::spinOnce();
					loop_rate.sleep();
				}
			
			}
			stop();
			std_velocity=temp_s;
			k_dist=temp_k;
			loop_rate.sleep();
			
			
			
			return 0;
		}

		heading_error=initial_theta-theta;

		// Some small threshold to take into account noise
		if(std::abs(heading_error)<heading_thres)
			heading_error=0.0;


		if(dist_wall(1) != -1){
			if(dist_wall(2) != -1){
				if(dist_wall(1) < dist_wall(2)){
					//ROS_INFO("IM SEEING A WALL TO THE LEFT!!");
					wall=1;
					if(dist_wall(1)<dist_ref)
						far_away_flag=0;

					for(int i=0; i<2; i++)
						ir_wall[i]=ir_readings[i+2];
				}else{
					//ROS_INFO("IM SEEING A WALL TO THE RIGHT!!");
					wall=2;
					if(dist_wall(2) < dist_ref)
						far_away_flag=0;

					for(int i=0; i<2; i++)
						ir_wall[i]=ir_readings[i+4];
				}
			}else{
				//ROS_INFO("IM SEEING A WALL TO THE LEFT!!");
				wall=1;
				if(dist_wall(1) < dist_ref)
					far_away_flag=0;

				for(int i=0; i<2; i++)
					ir_wall[i]=ir_readings[i+2];
			}
		}else if(dist_wall(2) != -1){
			//ROS_INFO("IM SEEING A WALL TO THE RIGHT!!");
			if(dist_wall(2) < dist_ref)
				far_away_flag=0;

			wall=2;
			for(int i=0; i<2; i++)
				ir_wall[i]=ir_readings[i+4];

		}else if(dist_wall(1)==-1 && dist_wall(2)==-1){ // Not seeing a wall -> we are risking getting misaligned with the maze's grid structure. Better slow down.

			if(ir_readings[2] < dist_thres/2 && ir_readings[4] < dist_thres/2){
				avoid_flag=1;
				if(ir_readings[2] < ir_readings[4]){
					ROS_INFO("IM SEEING SOMETHING TO THE LEFT AND I WANT TO AVOID IT!!");
					wall=1;
					for(int i=0; i<2; i++)
						ir_wall[i]=ir_readings[2];
				}else{
					ROS_INFO("IM SEEING SOMETHING TO THE RIGHT AND I WANT TO AVOIT IT!!");
					wall=2;
					for(int i=0; i<2; i++)
						ir_wall[i]=ir_readings[4];
				}
			}else if(ir_readings[2] < dist_thres/2){
				avoid_flag=1;
				ROS_INFO("IM SEEING SOMETHING TO THE LEFT AND I WANT TO AVOID IT!!");
				wall=1;
				for(int i=0; i<2; i++)
					ir_wall[i] = ir_readings[2];
			}else if(ir_readings[4] < dist_thres/2){
				avoid_flag=1;
				ROS_INFO("IM SEEING SOMETHING TO THE RIGHT AND I WANT TO AVOID IT!!");
				wall=2;
				for(int i=0; i<2; i++)
					ir_wall[i] = ir_readings[4];
			}else{
				//ROS_INFO("IM SEEING NOTHING!!");
				wall=0;
			}
		}

		if(wall){
			if(avoid_flag){
				if(ir_wall[0]<1.5){
					k_dist=temp_k;
					//k_dist=0.06; // OH THE SORROW
					//std_velocity=5.0; // OH SO SORRY
					u_theta=paralel_controller(wall,ir_wall,theta_ref,2.0,&last_E_r,&I_sum_r,&last_R_d,&I_sum_d);
					
					if(u_theta==ERROR_VAL){
					
						u_theta=0;
						
					}
					
					
					k_dist=0;
				}else{
					k_dist=0.0;
					u_theta=0;
				}
			}else{
				if(!far_away_flag){
					u_theta=paralel_controller(wall,ir_wall,theta_ref,dist_ref,&last_E_r,&I_sum_r,&last_R_d,&I_sum_d);
					
					if(u_theta==ERROR_VAL){
						
						u_theta=0;
						
					}
					
				}else{
					k_dist=0.0;
					if(wall!=wall_to_follow && (ir_wall[0]+ir_wall[1])>dist_ref)
						k_dist=0.0; // will not try to get close to the wall it's not following. But it is desirable to align with it
					u_theta=paralel_controller(wall,ir_wall,theta_ref,dist_ref,&last_E_r,&I_sum_r,&last_R_d,&I_sum_d);

					if(u_theta==ERROR_VAL){

						u_theta=0;

					}
					
				}
			}
		}else
			u_theta=0;		


		//Distance to wall < inf_thres ---> close! Reduce velocity
		if(wall_in_range(3,inf_thres,ir_readings)){

			if(ir_readings[0] > ir_readings[1])
				close_ir=ir_readings[1];
			else
				close_ir=ir_readings[0];

			if(flag_dist2break_1 == 1){
				initial_flag_dist2break_1 = close_ir;
				flag_dist2break_1 = 0;

				control_pub(std_velocity,u_theta);
			}else{
				//2 yields (1/2)*std_velocity... 1 gives 0 
				BreakingRatio_1 = ((inf_thres-close_ir)/(1.1*(inf_thres-dist_thres))); 
				control_pub(abs(std_velocity*( 1 - BreakingRatio_1 )),u_theta);
			}
		}else{
			control_pub(std_velocity,u_theta);
			flag_dist2break_1 = 1;
		}

		std_velocity=temp_s;
		//k_dist=temp_k;
		far_away_flag=1;
		loop_rate.sleep();
		ros::spinOnce();
		curr_dist=std::sqrt((x-i_x)*(x-i_x)+(y-i_y)*(y-i_y));
	}

	stop();
	std_velocity=temp_s;
	k_dist=temp_k;
	ROS_INFO("Finished the forward behavior successfully!\n");
	last_angle = 0;
	return 0;

}

/** STATE 2 
 * 	rotate: Implements the 'Rotate xº' behavior
 *	Sends a (v=0,w=k*error) message to the core, with the error being based on the odometry
 **/

int rotate(ros::Rate loop_rate){

	double heading_error=0.0, temp_thres;
	double init_theta=processed_theta;
	double init_error=heading_ref-(processed_theta-init_theta);
	int done=0, wall=1;
	double ir_wall[2]={0.0,0.0};
	double theta_ref=0.0, theta_meas=0.0, error_theta=0.0,u_theta=0.0;
	double I_sum=0.0, last_E=PID_INIT;

	ROS_INFO("Will rotate %.2f rad",heading_ref);
	// Rotation on-going
	stop();
	while(ros::ok() && !done){
		if(wall_in_range(1,inf_thres,ir_readings) || wall_in_range(2,inf_thres,ir_readings)){
			if(!wall_in_range(3,dist_thres,ir_readings))
				temp_thres=0.02;
			else
				temp_thres=heading_thres;
		}else{
			temp_thres=heading_thres;
		}
		heading_error=heading_ref-(processed_theta-init_theta);
		// If Rotation completed
		if(std::abs(heading_error) < temp_thres){
			done=1;
			stop();
			loop_rate.sleep();
			ros::spinOnce();
		}else{
			u_theta=PID_control(k_rotate,i_rotate,d_rotate,&I_sum, &last_E,heading_ref,processed_theta-init_theta);
			control_pub(0.0,u_theta+sign(heading_error)*cte_rotate);
			loop_rate.sleep();
			ros::spinOnce();
		}
	} 
	ROS_INFO("Finished rotating");
	last_angle = 0;
	return 0;
}

/** STATE 3
 *	forward_wall: Implements the 'Forward with wall' behavior
 *	Sends a (v>0, w=k*error) message to the core, with the error being based on the ir readings. Goes back to none when detects a corner or stops seeing the wall
 **/
int forward_wall(ros::Rate loop_rate){

	double ir_wall[2]={0.0,0.0}, close_ir=0.0;
	double theta_ref=0.0, theta_meas=0.0, error_theta=0.0,u_theta=0.0,u_dist=0.0; 
	double dist_meas=0.0, error_dist=0.0, avg_dist=0.0, temp_s, curr_dist, forward_distance,i_x,i_y;
	double I_sum_r=0.0, last_E_r=PID_INIT, I_sum_d=0.0, last_R_d=PID_INIT;
	int wall=1; // 1 - left side; 2 - right side
	double wall_dist=0.0;
	double BreakingRatio_3; //Variable used to compute the velocity to break proportional to distance

	if(wall_to_follow!=1 && wall_to_follow!=2){
		ROS_INFO("WALL_TO_FOLLOW IS %.2f!!!!!",wall_to_follow);
		getchar();
	}

	while(ros::ok()){

		if(stop_flag){
			stop();
			stop_flag=0;
			ROS_INFO("Object ahead!\n");
			return 0;
		}

		if(wall_in_range(3,dist_thres,ir_readings) || wall_in_range(4,cross_thres1,ir_readings)){ 		// check if obstacle ahead
			stop();
			last_angle=0;
			ROS_INFO("Stop! Obstacle ahead!\n");
			
			if(wall_in_range(4,cross_thres1,ir_readings)){ // will go back
				
				temp_s=std_velocity;
				std_velocity=-10.0;
				forward_distance=5.0;
				curr_dist = 0;
				i_x=x;
				i_y=y;
				while(curr_dist < forward_distance){
					
					control_pub(std_velocity,u_theta);
				
					curr_dist=std::sqrt((x-i_x)*(x-i_x)+(y-i_y)*(y-i_y));
					//ROS_INFO("MOVED %.2f!",curr_dist);
					ros::spinOnce();
					loop_rate.sleep();
				}
				
				std_velocity = temp_s;
			
			}
			
			
			
			return 0;
		}

		if (wall_in_range(1,inf_thres,ir_readings) && wall_to_follow==1){
			// check if left wall AND following left side
			ir_wall[0]=ir_readings[2];
			ir_wall[1]=ir_readings[3];
			wall_dist=dist_wall(wall_to_follow);
			ROS_INFO("\n Left wall\n"); 
		}else if(wall_in_range(2,inf_thres,ir_readings) && wall_to_follow==2){ 					
			// check if right wall AND following right side
			ir_wall[0]=ir_readings[4];
			ir_wall[1]=ir_readings[5];
			wall_dist=dist_wall(wall_to_follow);
			ROS_INFO("\n Right wall\n");
		}else{
			wall=0;
			last_angle = error_theta;
		}

		//Lost wall
		if(wall==0){
			stop();
			ROS_INFO("Lost wall!\n"); 
			return 0;
		}

		//Debug
		if(wall_dist==-1){
			stop();
			ROS_INFO("Error wall_dist==-1!"); 
			return 0;
		}

		//double * last_E_r, double * I_sum_r, double * last_R_d, double * I_sum_d

		u_theta=paralel_controller(wall_to_follow,ir_wall,theta_ref,dist_ref,&last_E_r,&I_sum_r,&last_R_d,&I_sum_d);
		
		if(u_theta==ERROR_VAL){
			
			stop();
			ROS_INFO("Lost wall!\n"); 
			return 0;

		}
		

		//Control velocity of the robot
		if(wall_in_range(3,inf_thres,ir_readings)){
			//Distance to wall <= inf_thres ---> close! Reduce velocity
			if(ir_readings[0] > ir_readings[1])
				close_ir=ir_readings[1];
			else
				close_ir=ir_readings[0];

			if(flag_dist2break_3 == 1){
				flag_dist2break_3 = 0;
				initial_flag_dist2break_3 = close_ir; 
				velocity_fw=std_velocity;
				control_pub(velocity_fw,u_theta);
			}else{
				BreakingRatio_3 = (inf_thres-close_ir)/(1.1*(inf_thres-dist_thres));	//2 yields (1/2)*std_velocity... 1 gives 0 
				velocity_fw=abs(std_velocity*( 1 - BreakingRatio_3 ));
				control_pub(velocity_fw,u_theta); // u_theta should be the result of a PID controller
			}
		}else{
			//Distance to wall > inf_thres ---> normal
			velocity_fw=std_velocity;
			control_pub(velocity_fw,u_theta); // u_theta should be the result of a PID controller
			flag_dist2break_3 = 1;
		}

		loop_rate.sleep();
		ros::spinOnce();
	}

}

void object_stop(theia_services::stop::ConstPtr msg){

	stop_flag=msg->stop;

}

///////////////////////////////////////////////////////////////////////////////////////////////////////////
// 
// MAIN() FUNCTION
//  
///////////////////////////////////////////////////////////////////////////////////////////////////////////

int main(int argc, char ** argv){

	ros::init(argc, argv, "control_motion");
	ros::NodeHandle n;
	ros::Rate loop_rate(freq);

	ask_logic = n.serviceClient<theia_services::MotionCommand>("/wall_follower/motion_command");
	ask_blind = n.serviceClient<theia_services::MotionCommand>("/blind_node/motion_command");

	vw_pub = n.advertise<core_control_motor::vw>("/control_motion/vw",1);
	
	odo_sub = n.subscribe("/core_sensors_odometry/odometry",1,odo_proc);
	ir_sub = n.subscribe("/core_sensors_ir/ir",1,ir_proc);
	params_sub = n.subscribe("/control_motion/params",1,update_params);
	stop_sub = n.subscribe("/control_motion/stop",1,object_stop);

	//ROS_INFO("Started the control_motion node");

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
		default:
			behavior = 0;
			break;
		}
	}

	return 0;

}


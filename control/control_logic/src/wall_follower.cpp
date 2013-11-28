#include <ros/ros.h>
#include <theia_services/MotionCommand.h>
#include <theia_services/object.h>
#include "control_logic/info.h"
#include <core_sensors/ir.h>
#include <theia_services/brain_wall.h>

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
double cross_thres1=10;
double cross_thres2=10;

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

// info stuff
char info_heading='E';
int info_wall=0;
ros::Publisher info_pub;


double ir[ir_size] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
double ir_raw[ir_size][median_size];

typedef struct history_struct{
	int driving_mode; //0:None 1:FWD 2:ROTATE 3:FOLLOW_W
	double driving_parameters; //None || Angle || Wall
}driving_history;

driving_history history[hist_size];

int flag_turning = 0;
int flag_avoid = 0;
int flag_object = 0;
int object_in_front = 0;
bool active=false;

///////////////////////////////////////////////////////////////////////////////////////////////////////////
// 
// MISC FUNCTIONS
//  
///////////////////////////////////////////////////////////////////////////////////////////////////////////

/* 
 * median: median filter the IR values
 * */
double median(double ir[median_size]){ // Simple median_size-value median filter. median_size should be odd
	double temp, ir_temp[median_size];

	for(int i=0; i < median_size; i++)
		ir_temp[i]=ir[i];

	for(int i=0; i < median_size; i++){
		for(int j=i+1; j<median_size; j++){
			if(ir[i]>ir[j]){
				temp=ir_temp[i];
				ir_temp[i]=ir_temp[j];
				ir_temp[j]=temp;
			}
		}
	}
	return ir_temp[(median_size-1)/2];
}

/* 
 * readIrData: updates the IR values obtained from the core, then filtering.
 * */
void readIrData(core_sensors::ir::ConstPtr ir_msg){

	//Shifting IR values to the immediate right cell
	for(int i=0; i<ir_size; i++){
		for(int j=0; j<2;j++){
			ir_raw[i][j+1]=ir_raw[i][j];
		}
	}

	//Filter
	for(int i=0; i<ir_size; i++){
		ir_raw[i][0]=ir_msg->dist[i];
		ir[i]=median(ir_raw[i]);
	}

}

/* 
 * readObjectData: updates the IR values obtained from the core, then filtering.
 * */
void readObjectData(theia_services::object::ConstPtr msg){

	object_in_front = msg -> object;

	if (object_in_front){
		flag_object = 1;
	}

}

/* 
 * initialize_ir_raw: initialize the IR values by assigning every cell to zero
 **/
void initialize_ir_raw(void){
	for(int i=0; i<ir_size; i++){
		for(int j=0; j<3; j++){
			ir_raw[i][j]=0;
		}
	}

	return;
}

/* 
 * turn_TYPE: possible turning options
 **/
void turn_left() {

	history[0].driving_mode=2;
	history[0].driving_parameters=PI/2;
	return;
}

void change_heading(char delta_heading){

	switch(info_heading){

	case 'E':
		if(delta_heading=='R')
			info_heading='S';
		else
			info_heading='N';
		break;
	case 'N':
		if(delta_heading=='R')
			info_heading='E';
		else
			info_heading='W';
		break;
	case 'W':
		if(delta_heading=='R')
			info_heading='N';
		else
			info_heading='S';
		break;
	case 'S':
		if(delta_heading=='R')
			info_heading='W';
		else
			info_heading='E';
		break;
	}


}


void turn_right() {

	history[0].driving_mode=2;
	history[0].driving_parameters=-PI/2;
	info_wall=-1;
	return;
}

double turn_random() {
	int random_var = 0; 
	int driving_var = 1;
	random_var=rand()%10; // random_var = {1, 10}
	double d_param = 0;

	info_wall=-1;

	if (random_var > 5){ 
		d_param=PI/2;
		change_heading('L');
	}else{
		d_param=-PI/2;
		change_heading('R');
	}
	return d_param;
}

/** 
 * turn_TYPE: possible turning options
 **/
void go_forward() {
	info_heading_ref = 0;
	drive_mode = 3;
	last_direction = 3;
	return;
}

/** 
 * initialize_history: initialize the history by assuming we have driving forward for infinite time
 **/
void initialize_history(void){

	int i;
	for (i=0; i < hist_size; i++){
		history[i].driving_mode=1;
		history[i].driving_parameters=forward_standard;
	}
	return;
}

/** 
 * shift_history: updates the history by shifting to the right all the cells in the vector
 **/
void shift_history(void){

	int i;
	for (i=hist_size; i>0; i--){
		history[i]=history[i-1];
	}
	//We are going to move forward if an error is detected. Instead of initializing to 1
	history[0].driving_mode=1;
	history[0].driving_parameters=forward_standard;

	return;
}

/* 
 * object_in_range: Checks from sensor camera if there is an object in front
 **/
int object_in_range(void){ 

	if(object_in_front){
		return 1;
	}else{
		return 0;
	}
}

/* 
 * wall_in_range: Checks for a nearby wall for a given threshold, thres. Side: 1 - Left, 2 - Right, 3 - front, 4- crossed
 **/
int wall_in_range(int side, double thres){ 

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
		if((ir[6] < cross_thres1 || ir[7] < cross_thres2) && ir[0] > front_max && ir[1] > front_max){
			return 1;
		}else{
			return 0;
		}
	default:
		return 1;
	}

}

/** 
 * closest_wall: returns the closest one of both walls 
 * wall = 1 (left); wall = 2 (right);	
 **/
int closest_wall(void){

	double closest_wall_left = 0.0;
	double closest_wall_right = 0.0;

	if ( !wall_in_range(1, side_max) && wall_in_range(2, side_max)){
		return 2;
	}

	else if ( wall_in_range(1, side_max) && !wall_in_range(2, side_max)){
		return 1;
	}

	else if ( wall_in_range(1, side_max) && wall_in_range(2, side_max)){
		if(ir[2] < ir[3]) 		// check for closest wall on left side
			closest_wall_left=ir[2];
		else
			closest_wall_left=ir[3];

		if(ir[4] < ir[5]) 		// check for closest wall on right side
			closest_wall_right=ir[4];
		else
			closest_wall_right=ir[5];

		if(closest_wall_left < closest_wall_right){ // check for closest wall
			return 1;
		}else{
			return 2;
		}

	}else { 
		//No wall in range ( !wall_in_range(1, side_max) && !wall_in_range(2, side_max))
		ROS_INFO("Error closest_wall = -1");
		return -1;
	}
}



/** 
 * closest_frontal_wall: returns the closest frontal wall with the cross sensors
 * wall = 1 (left); wall = 2 (right);	
 **/
/*int closest_frontal_wall(void){

	double closest_wall_left = 0.0;
	double closest_wall_right = 0.0;

	if (!wall_in_range(3, front_min) ){
		return 2;
	}

	//No wall in range ( !wall_in_range(1, side_max) && !wall_in_range(2, side_max))
	ROS_INFO("Error closest_wall = -1");
	return -1;
	}
}*/

/*
 * last_wall_followed: search last wall followed. Looks if history.driving_mode == 3 (FOLLOW_W)
 * It will be called just when 2 walls are seen on the sides. Then we have to decide which wall to follow.
 * Special case: No previous history
 */ 

int last_wall_followed(void){

	for (int i=1; i < hist_size; i++){
		// history[0] should not be accessed!; history[1] should have the last command!
		if (history[i].driving_mode == 3)
		{
			//history[0].driving_mode = 3;
			//history[0].driving_parameters = history[i].driving_parameters; //Keep the last wall followed
			return (int) history[i].driving_parameters;
		}else{
			//Default: Empty driving history
		}	

	}
	//If there is no last_wall_followed
	//ROS_INFO("last_wall_followed(void): There is no last_wall_followed in history. Return closest wall()");
	return closest_wall();

}

/*
 * consecutive_rotations: search in history number of consecutive rotations and then UPDATES the value of history[0].
 * It will be called just when a wall is seen in front and we hace two options to choose.Then we have to decide where to rotate
 */ 
int * consecutive_rotations(void){

	int rotations=0; 
	int direction=0;
	int i=0;
	int * return_vec=0;

	return_vec = new int[2];

	for (i=1; i<hist_size; i++){
		// history[0] should not be accessed!; history[1] should have the last command!
		if (history[i].driving_mode == 2){
			if (direction==1){
				if(history[i].driving_parameters>0)
					rotations++;
				else
					break;
			}else if(!direction){

				if(history[i].driving_parameters>0){
					direction=1;
				}else{
					direction=2;
				}				
				return_vec[1]=i;
				rotations++;
			}else if(direction==2){
				if(history[i].driving_parameters<0)
					rotations++;
				else
					break;
			}
		}

		if(rotations==3)
			break;

	}

	return_vec[0]=rotations;

	return return_vec;
}

/** 
 * 
 * rotate2_(not_)last_rotation: returns the direction of the last rotation (or the opposite) based on the history vector and the previous rotation
 * 
 **/
double rotate2_last_rotation(int history_idx){
	if(history[history_idx].driving_parameters==PI/2){
		change_heading('L');
		return PI/2;
	}else if(history[history_idx].driving_parameters==-PI/2){
		change_heading('R');
		return -PI/2;
	}else {
		//No previous rotations found in history vector
		switch(last_wall_followed()){
		case 1:
			change_heading('L');
			return PI/2;
			break;
		case 2:
			change_heading('R');
			return -PI/2;
			break;
		default:
			//No wall following detected in the past. No wall to follow. Misbehave by turning randomly.
			ROS_INFO("ERROR history[1].driving_parameters = %.2f",history[1].driving_parameters);
			return turn_random();
		}
	}
}

double rotate2_not_last_rotation(int history_idx){
	if(history[history_idx].driving_parameters==PI/2){
		change_heading('R');
		return -PI/2;
	}else if(history[history_idx].driving_parameters==-PI/2){
		change_heading('L');
		return PI/2;
	}else {
		//No previous rotations found in history vector
		switch(last_wall_followed()){
		case 1:
			change_heading('R');
			return -PI/2;
			break;
		case 2:
			change_heading('L');
			return PI/2;
			break;
		default:
			//No wall following detected in the past. No wall to follow. Misbehave by turning randomly.
			ROS_INFO("ERROR history[1].driving_parameters = %.2f",history[1].driving_parameters);
			return turn_random();
		}
	}
}

/** 
 * 
 * rotate2_(not_)last_wall: returns the direction of the last wall (or the opposite) based on the history vector and the previous rotation
 * 
 **/
double rotate2_last_wall(){
	if(last_wall_followed()==1){
		change_heading('L');
		return PI/2;
	}else if(last_wall_followed()==2){
		change_heading('R');
		return -PI/2;
	}else{
		//No wall following detected in the past. No wall to follow. Misbehave by turning randomly.
		ROS_INFO("RANDOM following last_wall_followed() = %d",last_wall_followed());
		return turn_random();
	}
}

double rotate2_not_last_wall(){
	if(last_wall_followed()==1){
		change_heading('R');
		return -PI/2;
	}else if(last_wall_followed()==2){
		change_heading('L');
		return PI/2;
	}else{
		//No wall following detected in the past. No wall to follow. Misbehave by turning randomly.
		ROS_INFO("RANDOM following last_wall_followed() = %d",last_wall_followed());
		return turn_random();
	}
}

void publish_info(){
	control_logic::info msg;

	msg.info_heading=info_heading;
	msg.info_wall=info_wall;

	info_pub.publish(msg);

}


///////////////////////////////////////////////////////////////////////////////////////////////////////////
// 
///////////////////////////////////////////////////////////////////////////////////////////////////////////
// 
// WALL FOLLOWING SECTION
//  
///////////////////////////////////////////////////////////////////////////////////////////////////////////
// 
///////////////////////////////////////////////////////////////////////////////////////////////////////////

/*
 * think: will update &res after a given &req
 * */
bool think(theia_services::MotionCommand::Request &req, theia_services::MotionCommand::Response &res){

	int history_idx = 0, *rot_count=0;
	bool turn=true;

	if(ir[0]==0 && ir[1]==0 && ir[2]==0 && ir[3]==0 && ir[4]==0 && ir[5]==0 && ir[6]==0 && ir[7]==0){
		res.B=0;
		return true;
	}

	if(!active){ // motion will have to ask somewhere else
		res.B=0;
		initialize_history();
		return true;

	}

	info_wall=0;
	//ros::Duration refresh(0.1);
	//refresh.sleep(); // wait a bit before sending new orders

	//Get empty space in history vector
	shift_history();

	if( !wall_in_range(3, front_min) || flag_avoid){ 
		//There is an evil wall at front 
		if(!flag_avoid){

			if ( (wall_in_range(4,cross_thres1)) || (wall_in_range(4,cross_thres2))   ){
				history[0].driving_mode = 2;
				history[0].driving_parameters=rotate2_not_last_wall();
				flag_avoid = 1;
			}		

		}else{
			//(flag_avoid)
			//If we are avoiding and object then we shouldn't see that object anymore
			if ( (wall_in_range(4,cross_thres1)) || (wall_in_range(4,cross_thres2))   ){
				//If we see a new object in the front while avoiding, then we sort of begin to avoid
				if(history[1].driving_mode == 1 || history[1].driving_mode == 2){
					history[0].driving_mode = 2;
					history[0].driving_parameters=rotate2_not_last_wall();
					flag_avoid = 1;
				}
			}else if(history[1].driving_mode == 2){
				flag_turning = 1;
				flag_avoid=0;
				history[0].driving_mode = 1;
				history[0].driving_parameters=forward_standard;
			}

		}

	}else{

		///////////////////////////////////////////////////
		//CASE 0: I am not seeing walls
		///////////////////////////////////////////////////

		if ( !wall_in_range(1, side_max) && !wall_in_range(2, side_max) && (!wall_in_range(3, front_min)          ) ){ // && !wall_in_range(4,cross_thres1) && !wall_in_range(4,cross_thres2))

			if(!flag_turning){

				if(history[1].driving_mode == 1){
					//Going forward trying to find a wall
					//Might want to go infinite distance
					history[0].driving_mode = 1;
					history[0].driving_parameters = forward_standard;
					flag_turning=0;
				}

				else if(history[1].driving_mode == 2){
					/*We rotated because we saw a wall in the front and after rotated 
						we stopped seeing it. We want now to go around it. FWD... then ROTATE*/
					history[0].driving_mode = 1;
					history[0].driving_parameters = forward_standard;
					flag_turning=1;
				}

				else if(history[1].driving_mode == 3){
					//I was following a wall and I lost it
					history[0].driving_mode = 1;
					history[0].driving_parameters = forward_standard;
					flag_turning=1;

				}else{
					//COMING FROM NOWHERE
					if(history[1].driving_mode==0){
						ROS_INFO("Coming from nowhere");
						history[0].driving_mode = 3;
						history[0].driving_parameters = closest_wall();					   //Follow the closest wall
						info_wall=closest_wall();
						flag_turning = 0;
					}else{
						ROS_INFO("Case 0a: Impossible case flag_turning=0 and coming driving_mode = %d", history[1].driving_mode);
					}
				}


			}else{
				// flag_turning = 1

				//Same code as in the case of at least 1 wall when I am detecting the wall I was not following
				if(history[1].driving_mode == 1){

					if(history[1].driving_parameters==forward_standard){
						history[0].driving_mode = 2;
						info_wall=-1;
						history[0].driving_parameters=rotate2_last_wall();

					}else if(history[1].driving_parameters==forward_medium){
						history[0].driving_mode = 2;
						info_wall=-1;
						history[0].driving_parameters=rotate2_last_rotation(2);

					}else if(history[1].driving_parameters==forward_extended){
						ROS_INFO("Case 0b: I did not find a wall to follow\nImpossible case history[1].driving_parameters = %.2f",history[1].driving_parameters);
					}else{
						ROS_INFO("Case 0b: Error: history[1].driving_parameters = %.2f",history[1].driving_parameters);
					}
				}

				else if(history[1].driving_mode == 2){
					history[0].driving_mode = 1;
					if(history[2].driving_parameters==forward_standard){
						history[0].driving_parameters = forward_medium;
					}else{
						history[0].driving_parameters = forward_extended;
						flag_turning = 0;
					}

				}else if(history[1].driving_mode == 3){
					ROS_INFO("Case 0b: Impossible case flag_turning=1 and coming driving_mode = %d", history[1].driving_mode);
				}else{
					ROS_INFO("Case 0b: Impossible case flag_turning=1 and coming driving_mode = %d", history[1].driving_mode);
				}

			}

			///////////////////////////////////////////////////
			//CASE 1: If I am seeing at least one wall AND there is no wall very close at the front
			///////////////////////////////////////////////////
		}else if (((wall_in_range(1, side_max) || wall_in_range(2, side_max)) ) && (!wall_in_range(3, front_min) && !(wall_in_range(4,cross_thres1) && !wall_in_range(4,cross_thres2)))){

			// flag_turning = 0
			if (!flag_turning){
				if(history[1].driving_mode == 1){
					//Follow the wall if I am not turning around
					history[0].driving_mode = 3;
					history[0].driving_parameters = last_wall_followed();					   
					info_wall=last_wall_followed();
					//Previous instruction correcting bug in the defined
					/*history[0].driving_parameters = closest_wall();					   
						info_wall=closest_wall();*/
					//END Previous instruction
					flag_turning=0;

				}else if(history[1].driving_mode == 2){
					//We might be in a corner after rotation
					history[0].driving_mode = 3;
					history[0].driving_parameters = last_wall_followed();					   
					info_wall=last_wall_followed();
					flag_turning=0;

				}else if(history[1].driving_mode == 3){
					//I was following a wall and I lost it. Start to turn around
					history[0].driving_mode = 1;
					history[0].driving_parameters = forward_standard;
					flag_turning=1;

				}else{
					//COMING FROM NOWHERE
					ROS_INFO("Case 1a: Impossible case flag_turning=0 and coming driving_mode = %d", history[1].driving_mode);
					history[0].driving_mode = 3;
					history[0].driving_parameters = closest_wall();					   
					info_wall=closest_wall();
					flag_turning = 0;
				}


			}else{
				// flag_turning = 1

				if (wall_in_range(last_wall_followed(), side_max)){
					//Then it means that I am seeing the wall I was following before I started turning
					history[0].driving_mode = 3;
					history[0].driving_parameters=last_wall_followed();
					info_wall=last_wall_followed();
					flag_turning = 0;	
				}else{

					if(history[1].driving_mode == 1){

						if(history[1].driving_parameters==forward_standard){
							history[0].driving_mode = 2;
							info_wall=-1;

							history[0].driving_parameters=rotate2_last_wall();


						}else if(history[1].driving_parameters==forward_medium){
							history[0].driving_mode = 2;
							info_wall=-1;
							history[0].driving_parameters=rotate2_last_rotation(2);

						}else if(history[1].driving_parameters==forward_extended){
							ROS_INFO("Case 1b: I did not find a wall to follow\nImpossible case history[1].driving_parameters = %.2f",history[1].driving_parameters);
						}else{
							ROS_INFO("Case 1b: Error: history[1].driving_parameters = %.2f",history[1].driving_parameters);
						}
					}

					else if(history[1].driving_mode == 2){
						history[0].driving_mode = 1;
						if(history[2].driving_parameters==forward_standard){
							history[0].driving_parameters = forward_medium;
						}else{
							history[0].driving_parameters = forward_extended;
							flag_turning = 0;
						}

					}else if(history[1].driving_mode == 3){
						ROS_INFO("Case 1b: Impossible case flag_turning=1 and coming driving_mode = %d", history[1].driving_mode);
					}else{
						ROS_INFO("Case 1b: Impossible case flag_turning=1 and coming driving_mode = %d", history[1].driving_mode);
					}
				}
			}

			///////////////////////////////////////////////////
			//CASE 2: If I am seeing at least one wall to the side AND there is a wall very close at the front or crossed
			///////////////////////////////////////////////////
		}else if (((wall_in_range(1, side_max) || wall_in_range(2, side_max)) ) && ((wall_in_range(3, front_min))        )){ //|| wall_in_range(4,cross_thres1) || wall_in_range(4,cross_thres2)

			// flag_turning = 0
			if (!flag_turning){
				if(history[1].driving_mode == 1){
					//e.g. when we finished turning mode and we face a wall
					history[0].driving_mode = 2;
					history[0].driving_parameters=rotate2_not_last_rotation(2);
					info_wall=-1;

				}else if(history[1].driving_mode == 2){
					//e.g. when we rotate in an internal corner in a narrow path
					history[0].driving_mode = 2;
					history[0].driving_parameters=rotate2_last_rotation(1);
					info_wall=-1;

				}else if(history[1].driving_mode == 3){
					//Internal corner
					history[0].driving_mode = 2;
					history[0].driving_parameters=rotate2_not_last_wall();
					info_wall=-1;

				}else{
					ROS_INFO("Case 2a: Impossible case flag_turning=0 and coming driving_mode = %d", history[1].driving_mode);
				}
			}

			// flag_turning = 1
			else{

				if (wall_in_range(last_wall_followed(), side_max)){
					//Then it means that I am seeing the wall I following before I started turning
					history[0].driving_mode = 2;
					history[0].driving_parameters=rotate2_not_last_wall();
					info_wall=-1;
					flag_turning = 0;	
				}

				else{
					if(history[1].driving_mode == 1){
						history[0].driving_mode = 2;
						info_wall=-1;
						history[0].driving_parameters=rotate2_last_wall();
						flag_turning = 0;					

					}else if(history[1].driving_mode == 2){
						history[0].driving_mode = 2;
						history[0].driving_parameters=rotate2_last_wall();
						info_wall=-1;
						flag_turning = 0;

					}else if(history[1].driving_mode == 3){
						ROS_INFO("Case 2b: Impossible case flag_turning=1 and coming driving_mode = %d", history[1].driving_mode);
					}else{
						ROS_INFO("Case 2b: Impossible case flag_turning=1 and coming driving_mode = %d", history[1].driving_mode);
					}
				}
			}

			///////////////////////////////////////////////////
			//CASE 3: If I am seeing no walls to the side AND there is a wall very close at the front
			///////////////////////////////////////////////////
		}else if (((!wall_in_range(1, side_max) && !wall_in_range(2, side_max)) ) && ((wall_in_range(3, front_min))                   )){ // || (wall_in_range(4,cross_thres1) || wall_in_range(4,cross_thres2))

			rot_count=consecutive_rotations(); 

			// flag_turning = 0
			if (!flag_turning){
				if(history[1].driving_mode == 1){

					if(rot_count[0]==3){
						history[0].driving_mode = 2;
						history[0].driving_parameters = -history[rot_count[1]].driving_parameters; //Change the sign of rotation
						info_wall=-1;
						if(history[rot_count[1]].driving_parameters==PI/2)
							change_heading('R');
						else
							change_heading('L');
					}else{
						history[0].driving_mode = 2;
						history[0].driving_parameters=rotate2_last_wall();
						info_wall=-1;
					}

				}else if(history[1].driving_mode == 2){
					//e.g., Turning in a corner in a narrow path
					history[0].driving_mode = 2;
					history[0].driving_parameters=rotate2_last_wall();
					info_wall=-1;
				}else if(history[1].driving_mode == 3){
					ROS_INFO("Case 3a: Error flag_turning has to be 1");

				}else{
					ROS_INFO("Case 3a: Impossible case flag_turning=0 and coming driving_mode = %d", history[1].driving_mode);
				}
			}

			// flag_turning = 1
			else{
				if(history[1].driving_mode == 1){
					history[0].driving_mode = 2;
					history[0].driving_parameters=rotate2_last_wall();
					info_wall=-1;
				}else if(history[1].driving_mode == 2){
					//e.g., Trying to access a very narrow gap when turning, and there is no space in front to go FWD
					history[0].driving_mode = 2;
					history[0].driving_parameters=rotate2_not_last_rotation(1);
					info_wall=-1;
				}else if(history[1].driving_mode == 3){
					ROS_INFO("Case 3: ERROR I cannot be in history[1].driving_mode = %d and flag=1",history[1].driving_mode);
				}else{
					ROS_INFO("Case 3: Impossible case flag_turning=1 and coming driving_mode = %d", history[1].driving_mode);
				}
			}

			delete [] rot_count;
		}

		///////////////////////////////////////////////////
		//CASE default: No defined case
		///////////////////////////////////////////////////
		else {
			ROS_INFO("\nI am not in any case =(");
		}

	}



	///////////////////////////////////////////////////
	///////////////////////////////////////////////////
	//Finished cases. Debug
	///////////////////////////////////////////////////
	///////////////////////////////////////////////////

	ROS_INFO("Last wall: %d",last_wall_followed());
	ROS_INFO("Turn flag: %d",flag_turning);

	printf("\n");

	switch(history[0].driving_mode){
	case 1:
		ROS_INFO("GO FORWARD");
		break;
	case 2:
		ROS_INFO("TURN %.2f",history[0].driving_parameters);
		break;
	case 3:
		ROS_INFO("FOLLOW WALL: %.0f",history[0].driving_parameters);
		break;
	default:
		//If an error is detected then the default behaviour is going forward as initialization and shifting function suggest
		history[0].driving_mode=1;
		history[0].driving_parameters=forward_standard;
		ROS_INFO("ERROR!! OMG! ERROR!! \n history[0].driving_parameters == -1");
		break;
	}

	//getchar();
	res.B = history[0].driving_mode;
	res.parameter = history[0].driving_parameters;
	publish_info();

	//loop_rate.sleep();
	//ros::spinOnce();

	//Update in history vector
	return true;
}

bool status(theia_services::brain_wall::Request &req, theia_services::brain_wall::Response &res){

	active=req.active;
	info_heading=req.heading;
	res.ok=true;

	return true;

}
///////////////////////////////////////////////////////////////////////////////////////////////////////////
// 
// MAIN() SECTION
//  
///////////////////////////////////////////////////////////////////////////////////////////////////////////

int main(int argc, char ** argv){

	ros::init(argc, argv, "wall_follower");
	ros::NodeHandle n;
	ros::Rate loop_rate(10);

	ros::ServiceServer motion_command = n.advertiseService("/wall_follower/motion_command", think); //Set up service server in this node
	ros::ServiceServer orders = n.advertiseService("/wall_follower/instructions", status);
	ros::Subscriber ir_data = n.subscribe("/core_sensors_ir/ir", 1, readIrData);
	//ros::Subscriber camera_data = n.subscribe("/core_sensors_ir/ir", 1, readCameraData);
	ros::Subscriber object_subs = n.subscribe<theia_services::object>("/control_logic/object",1,readObjectData);

	info_pub = n.advertise<control_logic::info>("/control_logic/info",1);

	initialize_history();
	initialize_ir_raw();


	while(ros::ok()){
		loop_rate.sleep();
		ros::spinOnce();
	}

	return 0;

}

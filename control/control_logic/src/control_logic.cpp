#include <ros/ros.h>
#include "control_logic/MotionCommand.h"
#include <core_sensors/ir.h>

///////////////////////////////////////////////////////////////////////////////////////////////////////////
// 
// VARIABLE DEFINITIONS
//  
///////////////////////////////////////////////////////////////////////////////////////////////////////////

const float PI = 3.1415926f;

//IR distance thresholds
double front_max = 20.0;
double front_min = 7.5;
double side_max = 20.0;
double side_min = 4.0;
double side_ref = 4.0;

int last_turn = 0;	//0 - null, 1 - left, 2 - right
int last_direction = 0; //0 - null, 1 - left, 2 - right, 3 - forward
double range_exc = 20.0;
double heading_ref = 0;
double drive_mode = 0;
double forward_standard = 20.0;
double forward_medium = 30.0;
double forward_extended = 35.0;
const int hist_size = 100;
const int ir_size = 8;
const int median_size = 3;

double ir[ir_size] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
double ir_raw[ir_size][3];

typedef struct history_struct{
	int driving_mode; //0:None 1:FWD 2:ROTATE 3:FOLLOW_W
	double driving_parameters; //None || Angle || Wall
}driving_history;

driving_history history[hist_size];

int flag_turning = 0; 

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
 * wall_in_range: Checks for a nearby wall for a given threshold, thres. Side: 1 - Left, 2 - Right, 3 - front
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
	default:
		return 1;
	}

}


/* 
 * turn_TYPE: possible turning options
 **/
void turn_left() {

	history[0].driving_mode=2;
	history[0].driving_parameters=PI/2;
	return;
}

void turn_right() {

	history[0].driving_mode=2;
	history[0].driving_parameters=-PI/2;
	return;
}

double turn_random() {
	int random_var = 0; 
	int driving_var = 1;
	random_var=rand()%10; // random_var = {1, 10}
	double d_param = 0;
	if (random_var > 5){ 
		d_param=PI/2;
	}else{
		d_param=-PI/2;
	}
	return d_param;
}

/** 
 * turn_TYPE: possible turning options
 **/
void go_forward() {
	heading_ref = 0;
	drive_mode = 3;
	last_direction = 3;
	return;
}

/** 
 * initialize_history: initialize the history by assigning every cell to zero
 **/
void initialize_history(void){

	int i;
	for (i=0; i < hist_size; i++){
		history[i].driving_mode=0;
		history[i].driving_parameters=0;
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
	history[0].driving_mode=0;
	history[0].driving_parameters=0.0;

	return;
}

/** 
 * closest_wall: returns the closest one of both walls 
 * wall = 1 (left); wall = 2 (right);	
 **/
int closest_wall(void){

	double closest_wall_left = 0.0;
	double closest_wall_right = 0.0;
	int wall_num=0;

	if(ir[2] < ir[3]) 		// check for closest wall on left side
		closest_wall_left=ir[2];
	else
		closest_wall_left=ir[3];

	if(ir[4] < ir[5]) 		// check for closest wall on right side
		closest_wall_right=ir[4];
	else
		closest_wall_right=ir[5];

	if(closest_wall_left < closest_wall_right){ // check for closest wall
		wall_num=1;
		return wall_num;
	}else{
		wall_num=2;
		return wall_num;
	}

}

/*
 * last_wall_followed: search last wall followed. Looks if history.driving_mode == 3 (FOLLOW_W)
 * It will be called just when 2 walls are seen on the sides. Then we have to decide which wall to follow.
 * Special case: No previous history
 */ 

int last_wall_followed(void){

	for (int i=1; i < hist_size; i++){
		// history[0] should be empty!; history[1] should have the last command!
		if (history[i].driving_mode == 3)
		{
			//history[0].driving_mode = 3;
			//history[0].driving_parameters = history[i].driving_parameters; //Keep the last wall followed
			return (int) history[i].driving_parameters;
		}else{
			//Default: Empty driving history
			return closest_wall();
		}	
	}

	//history[0].driving_parameters = closest_wall(); //Keep the last wall followed
	return 0;	

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
		// history[0] should be empty!; history[1] should have the last command!
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

///////////////////////////////////////////////////////////////////////////////////////////////////////////
// 
// BRAIN SECTION
//  
///////////////////////////////////////////////////////////////////////////////////////////////////////////

/*
 * think: will update &res after a given &req
 * */
bool think(control_logic::MotionCommand::Request &req, control_logic::MotionCommand::Response &res){

	int history_idx = 0, *rot_count=0;
	bool turn=true;

	if(ir[0]==0 && ir[1]==0 && ir[2]==0 && ir[3]==0 && ir[4]==0 && ir[5]==0 && ir[6]==0 && ir[7]==0){
		res.B=0;
		return true;
	}

	ros::Duration refresh(0.1);
	refresh.sleep(); // wait a bit before sending new orders

	//Get empty space in history vector
	shift_history();

	///////////////////////////////////////////////////
	//CASE 0: I am not seeing walls
	///////////////////////////////////////////////////

	if ( !wall_in_range(1, side_max) && !wall_in_range(2, side_max) && !wall_in_range(3, front_min) ){

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
					flag_turning = 0;
				}else{
					ROS_INFO("Impossible case flag_turning=0 and coming driving_mode = %d", history[1].driving_mode);
				}
			}


		}else{
			// flag_turning = 1
			// flag_turning = 1

			//Same code as in the case of at least 1 wall when I am detecting the wall I 
			//was not following
			if(history[1].driving_mode == 1){

				if(history[1].driving_parameters==forward_standard){
					history[0].driving_mode = 2;
					if(history[2].driving_parameters==PI/2){
						history[0].driving_parameters=PI/2;
					}else if(history[2].driving_parameters==-PI/2){
						history[0].driving_parameters=-PI/2;
					}else{
						ROS_INFO("ERROR I was not turning correctly. history[0].driving_parameters = %.2f",history[0].driving_parameters);
					}

				}else if(history[1].driving_parameters==forward_medium){
					history[0].driving_mode = 2;
					if(history[2].driving_parameters==PI/2){
						history[0].driving_parameters=PI/2;
					}else if(history[2].driving_parameters==-PI/2){
						history[0].driving_parameters=-PI/2;
					}else{
						ROS_INFO("ERROR I was not turning correctly. history[2].driving_parameters = %.2f",history[0].driving_parameters);
					}

				}else if(history[1].driving_parameters==forward_extended){
					ROS_INFO("\nI did not find a wall to follow\nImpossible case history[1].driving_parameters = %.2f",history[1].driving_parameters);
				}else{
					ROS_INFO("Error: history[1].driving_parameters = %.2f",history[1].driving_parameters);
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
				ROS_INFO("Impossible case flag_turning=1 and coming driving_mode = %d", history[1].driving_mode);
			}else{
				ROS_INFO("Impossible case flag_turning=1 and coming driving_mode = %d", history[1].driving_mode);
			}


			/*
			//flag_turning=1
			//flag_turning=1

			if(history[1].driving_mode == 1){
				//Check how was the parameter of going FWD
				if(history[1].driving_parameters==forward_standard){
					history[0].driving_mode = 2;
					if(history[2].driving_parameters==1){
						history[0].driving_parameters=PI/2;
					}else{
						history[0].driving_parameters=-PI/2;
					}	
				}else if(history[1].driving_parameters==forward_medium){
					if (wall_in_range(last_wall_followed(), side_max)){
						//I will follow a wall to the side I was following. It was a corner
						history[0].driving_mode = 3;
						history[0].driving_parameters=last_wall_followed();
					}else{
						//I will rotate again if I am in a U turn
						if(history[2].driving_parameters==PI/2){
							history[0].driving_parameters=PI/2;
						}else if(history[2].driving_parameters==-PI/2){
							history[0].driving_parameters=-PI/2;
						}else{
							//ERROR!
							ROS_INFO("ERROR IT IS NOT U-TURNING");
						}
					}
				}else if(history[1].driving_parameters==forward_extended){
					ROS_INFO("Impossible case history[1].driving_parameters = %.2f",history[1].driving_parameters);
				}else{
					ROS_INFO("Impossible case history[1].driving_parameters = %.2f",history[1].driving_parameters);
				}
			}

			else if(history[1].driving_mode == 2){
				//PROBLEM What if I was going FWD lost and I rotate and stop seeing a wall??

				history[0].driving_mode = 1;
				if(history[2].driving_mode==3){
					//*We rotated because we saw a wall in the front and after rotated 
					 //* we stopped seeing it. We want now to go around it. FWD... then ROTATE					
					history[0].driving_parameters = forward_standard;
				}else if(history[2].driving_mode==1){
					//Check parameters of FWD
					if(history[2].driving_parameters==forward_standard){
						history[0].driving_parameters = forward_medium;
					}else if(history[2].driving_parameters==forward_medium){
						history[0].driving_parameters = forward_extended;
						//We finish rotation
						flag_turning=0;
					}else if(history[2].driving_parameters==forward_extended){ // failed to find wall -> im lost
						history[0].driving_parameters = forward_standard;
						flag_turning=0;				
					}else{
						ROS_INFO("Impossible case history[2].driving_parameters = %.2f",history[2].driving_parameters);
					}
				}else{
					//Case where there is no previous info. Or maybe 2 consecutive rotations while turning around
					ROS_INFO("Impossible case history[2].driving_parameters = %.2f",history[2].driving_parameters);
				}

			}else if(history[1].driving_mode == 3){
				//We are starting to rotate!
				ROS_INFO("Impossible case: When turning we cannot come from history[1].driving_mode = %.2f",history[2].driving_mode);
				//history[0].driving_mode = 1;
				//history[0].driving_parameters = forward_standard;

			}else{
				ROS_INFO("Impossible case flag_turning=1. I have to come from somewhere. history[1].driving_mode = %d", history[1].driving_mode);
			}*/
		}



		///////////////////////////////////////////////////
		//CASE 1: If I am seeing at least one wall AND there is no wall very close at the front
		///////////////////////////////////////////////////
	}else if (((wall_in_range(1, side_max) || wall_in_range(2, side_max)) ) && (!wall_in_range(3, front_min))){

		// flag_turning = 0
		// flag_turning = 0
		if (!flag_turning){
			if(history[1].driving_mode == 1){
				//Follow the wall if I am not turning around
				history[0].driving_mode = 3;
				history[0].driving_parameters = closest_wall();					   //Follow the closest wall
				flag_turning=0;

			}else if(history[1].driving_mode == 2){
				//We might be in a corner
				history[0].driving_mode = 3;
				history[0].driving_parameters = last_wall_followed();					   //Follow the closest wall
				flag_turning=0;

			}else if(history[1].driving_mode == 3){
				//I was following a wall and I lost it. Start to turn around
				history[0].driving_mode = 1;
				history[0].driving_parameters = forward_standard;
				flag_turning=1;

			}else{
				//COMING FROM NOWHERE
				if(history[1].driving_mode==0){
					ROS_INFO("Coming from nowhere");
					history[0].driving_mode = 3;
					history[0].driving_parameters = closest_wall();					   //Follow the closest wall
					flag_turning = 0;
				}else{
					ROS_INFO("Impossible case flag_turning=0 and coming driving_mode = %d", history[1].driving_mode);
				}
			}


		}else{
			// flag_turning = 1
			// flag_turning = 1

			if (wall_in_range(last_wall_followed(), side_max)){
				/*Then it means that I am seeing the wall I following before I started turning*/
				history[0].driving_mode = 3;
				history[0].driving_parameters=last_wall_followed();
				flag_turning = 0;	
			}else{

				if(history[1].driving_mode == 1){

					if(history[1].driving_parameters==forward_standard){
						history[0].driving_mode = 2;
						if(history[2].driving_parameters==PI/2){
							history[0].driving_parameters=PI/2;
						}else if(history[2].driving_parameters==-PI/2){
							history[0].driving_parameters=-PI/2;
						}else{
							ROS_INFO("ERROR I was not turning correctly. history[2].driving_parameters = %.2f",history[2].driving_parameters);
						}

					}else if(history[1].driving_parameters==forward_medium){
						history[0].driving_mode = 2;
						if(history[2].driving_parameters==PI/2){
							history[0].driving_parameters=PI/2;
						}else if(history[2].driving_parameters==-PI/2){
							history[0].driving_parameters=-PI/2;
						}else{
							ROS_INFO("ERROR I was not turning correctly. history[2].driving_parameters = %.2f",history[0].driving_parameters);
						}

					}else if(history[1].driving_parameters==forward_extended){
						ROS_INFO("\nI did not find a wall to follow\nImpossible case history[1].driving_parameters = %.2f",history[1].driving_parameters);
					}else{
						ROS_INFO("Error: history[1].driving_parameters = %.2f",history[1].driving_parameters);
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
					ROS_INFO("Impossible case flag_turning=1 and coming driving_mode = %d", history[1].driving_mode);
				}else{
					ROS_INFO("Impossible case flag_turning=1 and coming driving_mode = %d", history[1].driving_mode);
				}
			}
		}

		///////////////////////////////////////////////////
		//CASE 2: If I am seeing at least one wall to the side AND there is a wall very close at the front
		///////////////////////////////////////////////////
	}else if (((wall_in_range(1, side_max) || wall_in_range(2, side_max)) ) && (wall_in_range(3, front_min))){

		if (!flag_turning){
			if(history[1].driving_mode == 1){
				//e.g. when we finished turning around and we face a wall
				history[0].driving_mode = 2;
				if(history[2].driving_parameters==PI/2){
					history[0].driving_parameters=-PI/2;
				}else if(history[2].driving_parameters==-PI/2){
					history[0].driving_parameters=PI/2;
				}else{
					ROS_INFO("ERROR I was not turning correctly. history[2].driving_parameters = %.2f",history[0].driving_parameters);
				}

			}else if(history[1].driving_mode == 2){
				//e.g. when we rotate in an internal corner in a narrow path
				history[0].driving_mode = 2;
				if(history[1].driving_parameters==PI/2){
					history[0].driving_parameters=PI/2;
				}else if(history[1].driving_parameters==-PI/2){
					history[0].driving_parameters=-PI/2;
				}else{
					ROS_INFO("ERROR I was not turning correctly. history[2].driving_parameters = %.2f",history[0].driving_parameters);
				}

			}else if(history[1].driving_mode == 3){
				//Internal corner
				history[0].driving_mode = 2;
				if(history[1].driving_parameters==1){
					history[0].driving_parameters=PI/2;
				}else if(history[1].driving_parameters==2){
					history[0].driving_parameters=-PI/2;
				}else{
					ROS_INFO("ERROR I was not turning correctly. history[1].driving_parameters = %.2f",history[0].driving_parameters);
				}

			}else{
				ROS_INFO("Impossible case flag_turning=0 and coming driving_mode = %d", history[1].driving_mode);
			}
		}


		else{

			if (wall_in_range(last_wall_followed(), side_max)){
				/*Then it means that I am seeing the wall I following before I started turning*/
				history[0].driving_mode = 2;
				if(last_wall_followed()==1){
					history[0].driving_parameters=-PI/2;
				}else if(last_wall_followed()==2){
					history[0].driving_parameters=PI/2;
				}else{
					ROS_INFO("ERROR I was following last_wall_followed() = %.2f",last_wall_followed());
				}
				flag_turning = 0;	
			}

			else{
				if(history[1].driving_mode == 1){
					history[0].driving_mode = 2;
					if(last_wall_followed()==1){
						history[0].driving_parameters=PI/2;
					}else if(last_wall_followed()==2){
						history[0].driving_parameters=-PI/2;
					}else{
						ROS_INFO("ERROR I was following last_wall_followed() = %.2f",last_wall_followed());
					}
					flag_turning = 0;					

				}else if(history[1].driving_mode == 2){
					history[0].driving_mode = 2;
					if(last_wall_followed()==1){
						history[0].driving_parameters=PI/2;
					}else if(last_wall_followed()==2){
						history[0].driving_parameters=-PI/2;
					}else{
						ROS_INFO("ERROR I was following last_wall_followed() = %.2f",last_wall_followed());
					}
					flag_turning = 0;

				}else if(history[1].driving_mode == 3){
					ROS_INFO("Impossible case flag_turning=1 and coming driving_mode = %d", history[1].driving_mode);
				}else{
					ROS_INFO("Impossible case flag_turning=1 and coming driving_mode = %d", history[1].driving_mode);
				}
			}
		}

		///////////////////////////////////////////////////
		//CASE 3: If I am seeing no walls to the side AND there is a wall very close at the front
		///////////////////////////////////////////////////
	}else if (((!wall_in_range(1, side_max) && !wall_in_range(2, side_max)) ) && (wall_in_range(3, front_min))){

		rot_count=consecutive_rotations(); 

		if (!flag_turning){
			if(history[1].driving_mode == 1){

				if(rot_count[0]==3){
					history[0].driving_mode = 2;
					history[0].driving_parameters = -history[rot_count[1]].driving_parameters; //Change the sign of rotation
				}else{
					history[0].driving_mode = 2;
					if(last_wall_followed()==1){
						history[0].driving_parameters=PI/2;
					}else if(last_wall_followed()==2){
						history[0].driving_parameters=-PI/2;
					}else if(last_wall_followed()==0){
						history[0].driving_parameters=turn_random();
						ROS_INFO("ERROR I was following last_wall_followed() = %d",last_wall_followed());
					}
				}
			}

			else if(history[1].driving_mode == 2){
				//Turning in a corner in a narrow path
				history[0].driving_mode = 2;
				if(last_wall_followed()==1){
					history[0].driving_parameters=PI/2;
				}else if(last_wall_followed()==2){
					history[0].driving_parameters=-PI/2;
				}else{
					ROS_INFO("ERROR I was following last_wall_followed() = %d",last_wall_followed());
				}

			}else if(history[1].driving_mode == 3){
				ROS_INFO("Error flag_turning has to be 1");
			}else{
				ROS_INFO("Impossible case flag_turning=0 and coming driving_mode = %d", history[1].driving_mode);
			}
		}

		else{
			if(history[1].driving_mode == 1){
				history[0].driving_mode = 2;
				if(last_wall_followed()==1){
					history[0].driving_parameters=PI/2;
				}else if(last_wall_followed()==2){
					history[0].driving_parameters=-PI/2;
				}else if(last_wall_followed()==0){
					history[0].driving_parameters=turn_random();
					ROS_INFO("ERROR I was following last_wall_followed() = %d",last_wall_followed());
				}

			}else if(history[1].driving_mode == 2){

				if(history[1].driving_parameters==PI/2){
					history[0].driving_parameters=PI/2;
				}else if(history[1].driving_parameters==-PI/2){
					history[0].driving_parameters=-PI/2;
				}else{
					ROS_INFO("ERROR I was not turning correctly. history[1].driving_parameters = %.2f",history[1].driving_parameters);
				}

			}else if(history[1].driving_mode == 3){
				ROS_INFO("ERROR I cannot be in history[1].driving_mode = %d and flag=1",history[1].driving_mode);
			}else{
				ROS_INFO("Impossible case flag_turning=1 and coming driving_mode = %d", history[1].driving_mode);
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

	///////////////////////////////////////////////////
	//Finished cases. Debug
	///////////////////////////////////////////////////

	/*ROS_INFO("Last wall: %.1f",last_wall_followed);
	ROS_INFO("Turn flag: %d",flag_turning);

	for(int i=1; i<10;i++){
		printf("Mode at i=%d : %d ",i,history[i].driving_mode);
	}
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
		ROS_INFO("ERROR");
		break;
	}*/

	//getchar();

	res.B = history[0].driving_mode;
	res.parameter = history[0].driving_parameters;

	//Update in history vector
	return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////
// 
// MAIN() SECTION
//  
///////////////////////////////////////////////////////////////////////////////////////////////////////////

int main(int argc, char ** argv){

	ros::init(argc, argv, "control_logic");
	ros::NodeHandle n;
	ros::Rate loop_rate(10);

	ros::ServiceServer motion_command = n.advertiseService("control_logic/motion_command", think); //Set up service server in this node
	ros::Subscriber ir_data = n.subscribe("/core_sensors_ir/ir", 1, readIrData);

	initialize_history();
	initialize_ir_raw();


	while(ros::ok()){
		loop_rate.sleep();
		ros::spinOnce();
	}

	return 0;

}

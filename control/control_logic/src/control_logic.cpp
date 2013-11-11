#include <ros/ros.h>
#include "control_logic/MotionCommand.h"
#include <core_sensors/ir.h>

double ir[8] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
double ir_raw[8][3];
const float PI = 3.1415926f;
double fwd_buffer = 10.0;
double sde_buffer = 20.0;
double crs_buffer = 30.0;
int last_turn = 0;	//0 - null, 1 - left, 2 - right
int last_direction = 0; //0 - null, 1 - left, 2 - right, 3 - forward
double range_exc = 20.0;
double heading_ref = 0;
double drive_mode = 0;

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

/* Function that should update, the IR values obtained from the core, after filtering*/
void readIrData(core_sensors::ir::ConstPtr ir_msg){

	for(int i=0; i<8; i++){
		for(int j=0; j<2;j++){
			ir_raw[i][j+1]=ir_raw[i][j];
		}
	}

	for(int i=0; i<8; i++){
		ir_raw[i][0]=ir_msg->dist[i];
		ir[i]=median(ir_raw[i]);
	}

}


void turn_left() {
	heading_ref = PI/2;  //set to turn left
	last_turn = 1;
	last_direction = last_turn;
	ROS_INFO("TURNING LEFT");
	return;
}

void turn_right() {
	heading_ref = -PI/2;  //set to turn right
	last_turn = 2;
	last_direction = last_turn;
	ROS_INFO("TURNING RIGHT");
	return;
}

void turn_around() {
	heading_ref = PI;
	ROS_INFO("TURNING AROUND");
	last_direction=0;
	return;
}

void go_forward() {
	heading_ref = 0;
	drive_mode = 3;
	last_direction = 3;
	ROS_INFO("GOING FORWARD");
	return;
}

bool try_turn() {      // This expression assesses whether the robot has room to turn lef or right and which direction it should turn.
	bool left = false;   // Where possible it will turn a different direction each time.
	bool right = false;
	double left_avg=0.0,right_avg=0.0;
  
	if(ir[2] > sde_buffer && ir[3] > sde_buffer) left = true;
	if(ir[4] > sde_buffer && ir[5] > sde_buffer) right = true;

	if(left == true && right == false) {
		drive_mode=2;
		turn_left();
		return true;
		
	} else if(left == false && right == true) {
		drive_mode=2;
		turn_right();
		return true;
		
	} else if(left == true && right == true) {
		
		// First try: always turn left if possible
		drive_mode=2;
		turn_left();
		return true;
		
		
		/*left_avg=(ir[2]+ir[3])/2;
		right_avg=(ir[4]+ir[5])/2;
		
		if(left_avg > right_avg){
			turn_left();
			return true;
		}else{
			turn_right();
			return true;
		}*/
		
		/*if(last_turn = 0 || last_turn == 2) {
		
			turn_left();
			return true;
			
		} else if(last_turn == 1) {
		
			turn_right();
			return true;
			
		}*/
		
	} else 
		return false;

}
bool think(control_logic::MotionCommand::Request &req, control_logic::MotionCommand::Response &res){
  
	bool turn;
	
	if(ir[0]==0 && ir[1]==0 && ir[2]==0 && ir[3]==0 && ir[4]==0 && ir[5]==0 && ir[6]==0 && ir[7]==0){
		res.B=0;
		return true;
	}

	ros::Duration refresh(0.1);

	refresh.sleep(); // wait a bit before sending new orders

	res.B=0; //default
	heading_ref = 0;
	drive_mode = 0;
	
	
	ROS_INFO("Stop Type: %d", req.stop_type);
  	
  	
  	if(ir[0] > fwd_buffer && ir[1] > fwd_buffer && last_direction!=3){
  		go_forward();
  	}else{
  		turn=try_turn();
  	}
  	
  	if(turn==false){
  		turn_around();
  	}
  	
  	
  	
	/*switch(req.stop_type) {	//depending on stop reason different code will run
		case 1: 
			drive_mode = 2;
			if(try_turn()) {  //if it's possible to turn left or right it will, direction based on opposite of previous turn
				break;
			} //else 
				//turn_around();	//if can't turn left or right robot will turn 180 degrees
			//break;
		case 2: 
			//if(last_direction == 3) {  // if last direction was forward
			if(last_direction!=3){
				if(try_turn()) {        // then lets see if we can turn
					drive_mode = 2;
					break;
				} else {
					go_forward();         // if for some reason we can't turn we'll just continue forward
				  	break;
				}
			}
		case 3: 
			if(ir[0] > fwd_buffer && ir[1] > fwd_buffer) { // if we can go forward we will
				go_forward();
				break;
			} else {                                      // else we'll try turning
				if(try_turn()) {
				  drive_mode = 2;
				  break;
				}
			}
		case 4: 
			if(ir[0] < range_exc || ir[1] < range_exc) {         // Basically if we're in open space we'll just find the nearest
				go_forward();                                      // wall and then start following it. If everything is out of range 
				break;                                             // then we'll zigzag until we find something.
				} else if(ir[2] < range_exc || ir[3] < range_exc) {
					drive_mode = 2;
					turn_left();
					break;
				} else if(ir[4] < range_exc || ir[5] < range_exc) {
					drive_mode = 2;
					turn_right();
					break;
				} else {
					if(last_direction == 3) {
					  if(try_turn()) {
					    drive_mode = 2;
					    break;
					  }
					} else {
						go_forward();
						break;
					}
				}
		default: 
			go_forward();

	}*/

	ROS_INFO("Drive mode is %d\n Heading ref is %.2f", drive_mode, heading_ref);


	res.heading_ref = heading_ref;
	res.B = drive_mode;

	return true;
}

int main(int argc, char ** argv){
  
  ROS_INFO("Control Logic (The Brain) has started... hold onto your butts");

  ros::init(argc, argv, "control_logic");
  ros::NodeHandle n;
  ros::Rate loop_rate(10);
  

  ros::ServiceServer motion_command = n.advertiseService("control_logic/motion_command", think); //Set up service server in this node
  ros::Subscriber ir_data = n.subscribe("/core_sensors_ir/ir", 1, readIrData);
  
  for(int i=0; i<8; i++)
	  for(int j=0; j<3; j++)
		  ir_raw[i][j]=0;

  while(ros::ok()){
	  loop_rate.sleep();
	  ros::spinOnce();
  }
	  

  return 0;

}

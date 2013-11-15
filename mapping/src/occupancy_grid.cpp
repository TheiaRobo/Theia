	// ################### The approach ##########################

	// 1: Get measurements (IR (core),Camera(Image node?), Wall(motion_control), Odomentry(motion Control))

	// 2: Calculate Current Pose (regardning the markow assumption)

	// 3: Calculate Lo (wighted prior of the occupancy grids odd ratio)

	// 4: Sensor fusion!

	 //4: Map the measurments to occupancy grid and find out if it should be occupied(funtions that are doing this Occupancy_grid_Mappning and Inverse_Range_Sensor_Model)
	//		4.1  occupancy grid 
	//			4.1.1 
	//		4.2 Sensor_model the area around the robot is only taken into account!
	//		4.3 
	//		4.1 




	// 5: Send visualization message to RVIZ (as an example)

	// 6: Find out if the Robot is driving towards an occupied Element! Send message to the Logic_Node



//#################   Sensors and more infomation  ##############
//Msg_odometry -> message from odometry 
//Msg_Core -> IR_sensors!
// Msg_Camera -> if needed!
//Msg_Wall -> good!

// Scaled Grid system?

// Occupied will be 1 and free will be 0 in the grid,

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <stdio.h>

//#include <stlib.h>
#include <cmath>
#include <core_control_motor/vw.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>
#include <vector>

#include <core_sensors/ir.h>

//#include <tf/transform_datatypes.h>




 //Initialize
const unsigned int x_matrix=100;
const unsigned int y_matrix=100;
const float resolution_matrix=0.001; // in meter!
float Current_Pose[1]; //x,y coordinates
int threshold_Occupied=90;

// for later used in the backward model!
// matrix distance elements from  the robot
// int Max_Sensor_Range_IR= 30; 
// int Max_Sensor_Range_Camera=30;
// int Max_Sensor_Range_Wall=15;

// for later used in both approches backward and forward occupancy models
// float Past_Wieght_L;
// float Current_Wieght_L;
// float Measurment_IR[8];


//int Occupancy_Grid[x_matrix][y_matrix]; //Occupancy_grid_Matrix maybe should be a float since 
std::vector<signed char>  Occupancy_Grid(x_matrix*y_matrix,50);
ros::Publisher occ_pub;
ros::Publisher map_pub;
ros::Subscriber	odometry_sub;
//ros::Subscriber IR_sub;


// Super simple since the other thing did not work
void Get_Readings_Odometry(nav_msgs::Odometry::ConstPtr odometry_msg){

	Current_Pose[0]=odometry_msg->pose.pose.position.x; //hopefully this will work directly
	Current_Pose[1]=odometry_msg->pose.pose.position.y;

	ROS_INFO("Odometry Coordinates(x,y): (%d,%d)",Current_Pose[0],Current_Pose[1]);
	//Start in the middle of the grid
	int x_grid=int (Current_Pose[0] + round(x_matrix/2));
	int y_grid=int(Current_Pose[1] + round(y_matrix/2));
	
	
	//put it to occupied in the grid more advanced is the forward model for occuancy
	// bacause we want to have valuse Between 0-100 depending on how likely it is that it is a obsticle
	
	Occupancy_Grid[x_grid*y_grid]=0;

	ROS_INFO("Grid coordinates that is change to occupied: (%d,%d)",x_grid,y_grid);
}

void Send_Message(){

	nav_msgs::OccupancyGrid occ_msg;
	nav_msgs::MapMetaData map_msg;


	
	occ_msg.header.stamp=ros::Time::now();
	occ_msg.header.frame_id= "/map";

	map_msg.map_load_time=ros::Time::now();
	map_msg.resolution=resolution_matrix;
	map_msg.width=x_matrix;
	map_msg.height=y_matrix;
	//occ_msg.MapMetaData.geometry_msgs.Point
	//occ_msg.MapMetaData.geometry_msgs.Quaternion

	occ_msg.info=map_msg;
	//PROBLEM NOT WORKNING!!!!!!!! :@ :@
	//occ_msg.data = Occupancy_Grid;
	ROS_INFO("OK");


	occ_msg.data=Occupancy_Grid;


	ROS_INFO("OK?");

	occ_pub.publish(occ_msg);
	map_pub.publish(map_msg);
	ROS_INFO("Publish: occ and map");


}


// Did this stuff below maybe we could use it.... 12/11/2013  jimmy

/* not needed

void Check_If_Occupied(){
for(int x_count=0,x_count <x_matrix , x_count++){
	for(int y_count=0,y_count <y_matrix , y_count++){
		if (Occupancy_Grid[x_count][y_count]  < threshold_Occupied){
				// tell the logic node to do take action
			ROS_INFO("This position is occupied! [X,Y]=[%.2f,%.2f]",x_count,y_count); //debug message
		}
		else {
			// Send "everything is fine"!!! Maybe unessery 
			ROS_INFO("This position is NOT occupied! [X,Y]=[%.2f,%.2f]",x_count,y_count);	// debug message
		}
		
	}}
	
}

//################################## things for later ###################################
/*
void Current_pose(){
/* important notes
should start in the middle of the maze

*/


 /* Kalman Filter  that i did last year.... The implementation of this could get more  accurent measurments
 	Value = newValue - bias;
	Current += dt * Value;
	COV[0][0] += dt * (dt*COV[1][1] - COV[0][1] - COV[1][0] + R_current);
	COV[0][1] -= dt * COV[1][1];
	COV[1][0] -= dt * COV[1][1];
	COV[1][1] += R_bias * dt;
	H = COV[0][0] + Q_measure;
	K[0] = COV[0][0] / H;
	K[1] = COV[1][0] / H;
	y = Zk - current;
	current += K[0] * y;
	bias += K[1] * y;
	COV[0][0] -= K[0] * COV[0][0];
	COV[0][1] -= K[0] * COV[0][1];
	COV[1][0] -= K[1] * COV[0][0];
	COV[1][1] -= K[1] * COV[0][1];
    return current;
    */
    /*
  }

 void Get_Readings_IR(){ 

	for(int i=0; i<8; i++){
		Measurments[i]=ir_msg->dist[i];
}

void Sensor_Fusion(){
	// merge all  measurments together
	 
	//Multi-sensor fusion
}
}

void Backward_Occupancy_grid_Mappning(Past_Wieght_L, Current_Pose){
for(int X=Current_Pose[1]- Max_Sensor_Range_IR, X<=Current_Pose[1]+Max_Sensor_Range_IR,  X++) // covers the nearest area in X
	{
	for(int X=Current_Pose[2]- Max_Sensor_Range_IR, Y<=Current_Pose[2]+Max_Sensor_Range_IR,  Y++) // covers the nearest area in X
	{
		if (Occupancy_Grid[X][Y])
		{
			Current_Weight_L= Past_Wieght_L + Inverse_Range_Sensor_Model(Occupancy_grid[X][Y])-Lo;

		}
		else{ Current_}
			Current_Weight_L=Past_Wieght_L;

	}}
	return Current_Weight_L;
}

void Inverse_Range_Sensor_Model(MAP, Current_Pose, Measurments)
{
	Current_Pose
	float r = sqrt(exp2(MAP[X][Y]-Current_Pose[1])+exp2(MAP[X][Y]-Current_Pose[2]));
	float phiX= atan2(MAP[X][Y]-Current_Pose[1]);
	float phiY= atan2(MAP[X][Y]-Current_Pose[2]);
	// not finnished

}

void Forward_Occupancy_Grid_Mapping(Current_Pose, Measurments)
{

float LO=log(0.07/0.93) // chance that it is occupied 7% and 93% that is not occupied

for(int X=0, X<x_matrix,  X++) // covers the nearest area in X
	{
	for(int Y, Y<_matrix,  Y++) // covers the nearest area in X
	{
		Occupancy_Grid[X][Y]= max_element(k LO + measurement_model)

//not finnished see report thrun.occ.pdf in dropbox
	/*
		Will fill in the rest!
	*/

//}



// ########################## MAIN ################################

int main(int argc, char **argv)
{
	ros::init(argc, argv, "occupancy_grid");
	ros::NodeHandle n;

    
	//IR_sub = n.subscribe("/core_sensors_ir/ir",1,);
	odometry_sub = n.subscribe("/core_sensors_odometry/odometry",1,Get_Readings_Odometry);
	
	occ_pub = n.advertise<nav_msgs::OccupancyGrid>("/occupancy_grid/occ",1);
	map_pub = n.advertise<nav_msgs::MapMetaData>("occupancy_grid/map",1);

	//Pose_pub = n.advertise<occupancy_grid::pose>("/occupancy_grid/pose",1);
	//Camera_sub = n.subscribe("", 1000, Receive_Camera_readings);
	//Motion_control_wall= n.subcribe("",1000,Receive_Wall);

	ROS_INFO("Started the occupancy_grid Node");

	ros::Rate loop_rate(100);

	struct timeval start, end;
	while(ros::ok()){
		ROS_INFO("Working? ROS OK!");
		Send_Message();

		loop_rate.sleep();
		ros::spinOnce();

	}
	return 0;
}
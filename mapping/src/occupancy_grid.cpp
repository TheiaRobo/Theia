

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <stdio.h>
#include <cmath>
#include <core_control_motor/vw.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>
#include <control_logic/info.h>
#include <vector>




 //Initialize
const float PI=3.1415926f;
const double resolution_matrix=0.02; // in meter!
const int x_matrix=0.1/resolution_matrix; 
const int y_matrix=0.1/resolution_matrix;

// robot size: 2*delta_x+1, 2*delta_y+1
const int robot_delta_x=5;
const int robot_delta_y=5;


int x_Current_Pose=round(x_matrix/2); //x,y coordinates
int y_Current_Pose=round(y_matrix/2); //x,y coordinates
const int freq=100;


// high level info

int wall=0;
double heading=0.0;


std::vector<signed char>  Occupancy_Grid(x_matrix*y_matrix,50);
ros::Publisher occ_pub;
ros::Publisher map_pub;
ros::Subscriber	odometry_sub;
ros::Subscriber	camera_sub;
ros::Subscriber	logic_sub;


/* cell_round
 * Rounds the received value to the nearest cell coordinate
 */
double cell_round(double val){
	double cm_res=resolution_matrix*100;
	
	return round(val/cm_res);
	
}

// Super simple since the other thing did not work
void Get_Readings_Odometry(nav_msgs::Odometry::ConstPtr odometry_msg){

	double x_Pose_Odometry=cell_round(odometry_msg->pose.pose.position.x*100); //odometry is comming in meters
	double y_Pose_Odometry=cell_round(odometry_msg->pose.pose.position.y*100);
	
	//Start in the middle of the grid
	x_Current_Pose = (x_Pose_Odometry + round(x_matrix/2));
	y_Current_Pose = (y_Pose_Odometry + round(y_matrix/2));
}


void place_map(int x_position, int y_position, int delta_x, int delta_y, int val) {

	for (int x=(x_position-delta_x)*x_matrix;x<=(x_position+delta_x)*x_matrix;x+=x_matrix){
		for (int y=y_position-delta_y;y<=y_position+delta_y;y++){

			Occupancy_Grid[x+y]=val;

		}
	}
	
	
}


void Send_Message(){

	nav_msgs::OccupancyGrid occ_msg;
	nav_msgs::MapMetaData map_msg;

	occ_msg.header.stamp=ros::Time::now();
	occ_msg.header.frame_id= "/mapping";

	map_msg.map_load_time=ros::Time::now();
	map_msg.resolution=resolution_matrix;
	map_msg.width=x_matrix;
	map_msg.height=y_matrix;

	occ_msg.info=map_msg;
	

	occ_msg.data=Occupancy_Grid;

	occ_pub.publish(occ_msg);

}

void Receive_Camera(int x_position, int y_position) {

}
void Get_Motion_Info(control_logic::info::ConstPtr logic_msg) {
	
	wall=logic_msg->info_wall;
	heading=logic_msg->info_heading;
	
	ROS_INFO("Heading: %.2f Wall: %d",heading,wall);
	
}


// ########################## MAIN ################################

int main(int argc, char **argv)
{
	ros::init(argc, argv, "occupancy_grid");
	ros::NodeHandle n;

    
	odometry_sub = n.subscribe("/core_sensors_odometry/odometry",1,Get_Readings_Odometry);
	logic_sub = n.subscribe("/control_logic/info",1,Get_Motion_Info);
	
	occ_pub = n.advertise<nav_msgs::OccupancyGrid>("/occupancy_grid/occ",1);
	map_pub = n.advertise<nav_msgs::MapMetaData>("occupancy_grid/map",1);


	ROS_INFO("Started the occupancy_grid Node");

	ros::Rate loop_rate(freq);

	struct timeval start, end;
	
	while(ros::ok()){

		place_map(x_Current_Pose,y_Current_Pose,robot_delta_x,robot_delta_y,0);
		Send_Message();

		loop_rate.sleep();
		ros::spinOnce();


	}
	return 0;
}
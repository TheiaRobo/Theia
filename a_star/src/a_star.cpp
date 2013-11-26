#include "ros/ros.h"
#include "std_msgs/String.h"
#include <stdio.h>
#include <cmath>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>

#include <vector>


const int black=100;
const int blue=75;
const int gray=50;
const int white=0;


std::vector<signed char>  Occupancy_Grid(10*10,blue);


	
	
	

/*
// Sends the corrected map on request
bool provide_map(theia_services::mapsrv::Request &req, theia_services::mapsrv::Response &res){
	
	nav_msgs::OccupancyGrid Correct_Map_Msg;

	Correct_Map_Msg.header.stamp=ros::Time::now();
	Correct_Map_Msg.header.frame_id= "/mapping";

	Correct_Map_Msg.info.map_load_time=ros::Time::now();
	Correct_Map_Msg.info.resolution=resolution_matrix;
	Correct_Map_Msg.info.width=x_matrix;
	Correct_Map_Msg.info.height=y_matrix;

	Correct_Map(robot_delta_x,robot_delta_y);
	
	Correct_Map_Msg.data=Corrected_Map;
	res.map=Correct_Map_Msg;
	
	return true;
	
}*/

// ########################## MAIN ################################


int * find_closest(int x_i, int y_i, std::vector<signed char> matrix_array, int val_to_find){
	int lateral_size=std::sqrt(matrix_array.size());
	std::vector<std::vector<signed char> >matrix(lateral_size);
	int goal_coords[2]={0,0};
	
	
	// Conversion to matrix
	for(int i=0; i < lateral_size; i++){
	
		matrix[i].resize(lateral_size,blue);
	
	}
	for(int x=0; x < lateral_size; x++){
		for(int y=0; y < lateral_size; y++){
			matrix[x][y]=matrix_array[x+y*lateral_size];
		}
	}
	
	// values different from white and val_to_find are obstacles
	
	for(int x=0; x < lateral_size; x++){
		for(int y=0; y < lateral_size; y++){
			if(matrix[x][y]!=white && matrix[x][y]!=val_to_find){
				matrix[x][y]=black;
			}else if(val_to_find!=gray && matrix[x][y]==val_to_find){
				goal_coords[0]=x;
				goal_coords[1]=y;
			}
		}
	}
	
	
	// A star?
	
	
	
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "a_star");
	ros::NodeHandle n;

	
	/*ros::ServiceServer map_sender = n.advertiseService("/mapping/ProcessedMap", provide_map);*/


	ROS_INFO("Started the a_star Node");

	ros::Rate loop_rate(1);

	while(ros::ok()){

		find_closest(2,3,Occupancy_Grid,gray);
		loop_rate.sleep();
		ros::spinOnce();


	}
	return 0;
}

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <stdio.h>
#include <cmath>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>
#include <signal.h>

#include <vector>
#include "sets.h"


const int black=100;
const int blue=75;
const int gray=50;
const int white=0;
int execute=1;


std::vector<signed char>  Occupancy_Grid(20*20,white);


void trap(int signal){ 
	execute = 0; 
}


int * find_closest(int x_i, int y_i, std::vector<signed char> matrix_array, int val_to_find){
	int lateral_size=std::sqrt(matrix_array.size());
	std::vector<std::vector<signed char> >matrix(lateral_size);
	int goal_coords[2]={NO_VAL,NO_VAL},init_coords[2],cost=0;
	
	
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
	
	
	// A*
	
	init_coords[0]=x_i;
	init_coords[1]=y_i;
	
	if(goal_coords[0]!=NO_VAL)
		cost=std::abs(goal_coords[0]-init_coords[0])+std::abs(goal_coords[1]-init_coords[1]); // Manhattan distance
	
	search_set closedset;
	search_set openset(init_coords,cost);
	search_set came_from;
	node current,n;
	int i=0;
	
	signal(SIGINT, &trap);
	
	while(!openset.isempty() && execute){
		i++;
		
		ROS_INFO("iter: %d",i);
		
		current=openset.pop_best();
		if(goal_coords[0]==-1){
			if(matrix[current.coords[0]][current.coords[1]]==val_to_find){
				ROS_INFO("Found solution at %d %d!",current.coords[0],current.coords[1]);
				return 0;
			}
		}else if(current.coords[0]==goal_coords[0] && current.coords[1]==goal_coords[1]){
			ROS_INFO("Found solution at %d %d!",current.coords[0],current.coords[1]);
			return 0;
		}
		
		closedset.push_node(current.coords,current.came_from,current.cost);
		
		
		// FOR EACH NEIGHBOUR
		
		if(current.coords[0]-1>=0){
			n.coords[0]=current.coords[0]-1;
			n.coords[1]=current.coords[1];
			n.came_from[0]=current.coords[0];
			n.came_from[1]=current.coords[1];
			if(goal_coords[0]!=NO_VAL)
				n.cost=current.cost+1;
			else
				n.cost=current.cost+1+std::abs(goal_coords[0]-init_coords[0])+std::abs(goal_coords[1]-init_coords[1]);
			if(!closedset.check_if_in_set(n.coords)&& !openset.check_if_in_set(n.coords)) // checking the openset should NOT be done ...
				openset.push_node(n.coords,n.came_from,n.cost);
		}
		
		if(current.coords[1]-1>=0){
			n.coords[0]=current.coords[0];
			n.coords[1]=current.coords[1]-1;
			n.came_from[0]=current.coords[0];
			n.came_from[1]=current.coords[1];
			if(goal_coords[0]!=NO_VAL)
				n.cost=current.cost+1;
			else
				n.cost=current.cost+1+std::abs(goal_coords[0]-init_coords[0])+std::abs(goal_coords[1]-init_coords[1]);
			if(!closedset.check_if_in_set(n.coords)&& !openset.check_if_in_set(n.coords))
				openset.push_node(n.coords,n.came_from,n.cost);
		}
		
		if(current.coords[0]+1<lateral_size){
			n.coords[0]=current.coords[0]+1;
			n.coords[1]=current.coords[1];
			n.came_from[0]=current.coords[0];
			n.came_from[1]=current.coords[1];
			if(goal_coords[0]!=-1)
				n.cost=current.cost+1;
			else
				n.cost=current.cost+1+std::abs(goal_coords[0]-init_coords[0])+std::abs(goal_coords[1]-init_coords[1]);
			if(!closedset.check_if_in_set(n.coords)&& !openset.check_if_in_set(n.coords))
				openset.push_node(n.coords,n.came_from,n.cost);
		}
		
		if(current.coords[1]+1<lateral_size){
			n.coords[0]=current.coords[0];
			n.coords[1]=current.coords[1]+1;
			n.came_from[0]=current.coords[0];
			n.came_from[1]=current.coords[1];
			if(goal_coords[0]!=-1)
				n.cost=current.cost+1;
			else
				n.cost=current.cost+1+std::abs(goal_coords[0]-init_coords[0])+std::abs(goal_coords[1]-init_coords[1]);
			if(!closedset.check_if_in_set(n.coords) && !openset.check_if_in_set(n.coords))
				openset.push_node(n.coords,n.came_from,n.cost);
		}
		
	}
	
	ROS_INFO("Did not find anything :(");
	return 0;
	
	
	
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "a_star");
	ros::NodeHandle n;

	
	/*ros::ServiceServer map_sender = n.advertiseService("/mapping/ProcessedMap", provide_map);*/


	ROS_INFO("Started the a_star Node");

	ros::Rate loop_rate(1);
	
	Occupancy_Grid[19+19*20]=2;
	
	//while(ros::ok()){

		find_closest(0,0,Occupancy_Grid,2);
		//loop_rate.sleep();
		//ros::spinOnce();


	//}
	return 0;
}

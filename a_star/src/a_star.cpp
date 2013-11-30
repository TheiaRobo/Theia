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
std::vector<int> commands;
std::vector<double> params;


void trap(int signal){ 
	execute = 0; 
}

// fills in the commands and params vectors based on the path to the solution
void recover_path(node goal, search_set node_set, int init[2]){
	node current = goal;
	std::vector<node> path_list;
	
	while(current.coords[0]!=init[0] || current.coords[1]!=init[1]){
		
		path_list.push_back(current);
		current=node_set.pop_requested(current.came_from);
	
	}
	
	// from path, get commands


}

node create_node(int coords[2], double t_f, double t_g, int came_from[2]){

	node n;
	n.coords[0]=coords[0];
	n.coords[1]=coords[1];
	n.t_f=t_f;
	n.t_g=t_g;
	n.came_from[0]=NO_VAL;
	n.came_from[1]=NO_VAL;
	
	return n;
}

double heur(int coords[2], int goal_coords[2]){
	
	double t_h=0;
	
	if(goal_coords[0]!=NO_VAL){
		t_h = std::abs(coords[0]-goal_coords[0])+std::abs(coords[1]-goal_coords[1]);
		return t_h;
	}else{ // no goal, no heuristic
		return 0;
	}


}


int * find_closest(int x_i, int y_i, std::vector<signed char> matrix_array, int val_to_find){
	int lateral_size=std::sqrt(matrix_array.size());
	std::vector<std::vector<signed char> >matrix(lateral_size);
	int goal_coords[2]={NO_VAL,NO_VAL},coords[2],from[2],cost=0;
	double t_f=0, t_g=0, t_h=0;
	
	
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
	
	
	node current,n;
	
	coords[0]=x_i;
	coords[1]=y_i;
	from[0]=NO_VAL;
	from[1]=NO_VAL;
	
	t_g=0;
	t_h=heur(coords,goal_coords);
	t_f = t_g + t_h;
	
	current=create_node(coords,t_f,t_g,from);
	
	
	search_set closedset;
	search_set openset(current);
	search_set came_from;
	search_set nodes_set(current);
	
	int i=0;
	
	signal(SIGINT, &trap);
	
	while(!openset.isempty() && execute){
		i++;
		
		ROS_INFO("iter: %d",i);
		
		current=openset.pop_best();
		
		
		if(goal_coords[0]==-1){
			if(matrix[current.coords[0]][current.coords[1]]==val_to_find){
				ROS_INFO("Found solution at %d %d!",current.coords[0],current.coords[1]);
				return 0; // later, the path
			}
		}else if(current.coords[0]==goal_coords[0] && current.coords[1]==goal_coords[1]){
			ROS_INFO("Found solution at %d %d!",current.coords[0],current.coords[1]);
			return 0;
		}
		
		closedset.push_node(current);
		
		
		// FOR EACH NEIGHBOUR
		
		if(current.coords[0]-1>=0){
			
			coords[0]=current.coords[0]-1;
			coords[1]=current.coords[1];
			
			t_h=heur(n.coords,goal_coords);
			t_g = current.t_g + 1;
			t_f = t_g + t_h;
			
			if(nodes_set.check_if_in_set(coords)){
				
				n = nodes_set.pop_requested(n.coords); // so I can keep track of the f cost
				
			}else{
				n=create_node(coords,t_f,t_g,current.coords);
				
			}
			
			if(!closedset.check_if_in_set(n.coords) || t_f < n.t_f ){
				
				if(!openset.check_if_in_set(n.coords))
					openset.push_node(n);
				else{ // update the value of the node in openset
					openset.pop_requested(n.coords);
					openset.push_node(n);
				}
				
				nodes_set.push_node(n);
				
			}
			
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

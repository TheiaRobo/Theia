#include "ros/ros.h"
#include "std_msgs/String.h"
#include <stdio.h>
#include <cmath>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>
#include <signal.h>
#include <path_planner/path_srv.h>

#include <vector>
#include "sets.h"


const int black=100;
const int blue=75;
const int gray=50;
const int white=0;
int execute=1;

std::vector<signed char>  Occupancy_Grid;
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
	
	//ROS_INFO("Created node with t_f=%.2f and t_g=%.2f",t_f,t_g);
	
	return n;
}

double heur(int coords[2], int goal_coords[2]){
	
	double t_h=0;
	
	if(goal_coords[0]!=NO_VAL){
		t_h = std::abs(coords[0]-goal_coords[0])+std::abs(coords[1]-goal_coords[1]);
		return 1.05*t_h;
	}else{ // no goal, no heuristic
		return 0;
	}


}

std::vector<node> retrieve_path(node goal, search_set * closed){
	
	node current=goal;
	std::vector<node> return_list(1,goal);
	
	while(current.came_from[0]!=NO_VAL){
		
		current=(*closed).pop_requested(current.came_from);
		return_list.push_back(current);
		
		
	}
	
	
	return return_list;
	
	
}


std::vector<node> find_closest(int x_i, int y_i, std::vector<signed char> matrix_array, int val_to_find){
	int lateral_size=std::sqrt(matrix_array.size());
	std::vector<std::vector<signed char> >matrix(lateral_size);
	int goal_coords[2]={NO_VAL,NO_VAL},coords[2],from[2],cost=0;
	std::vector<node> return_error;
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
	ROS_INFO("Will start A*. Goal found in pos (%d,%d). Press any key to continue...",goal_coords[0]+1,goal_coords[1]+1);
	//getchar();
	
	
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
		
		//ROS_INFO("iter: %d",i);
		
		current=openset.pop_best();
		
		/*ROS_INFO("Will evaluate node (%d,%d).",current.coords[0],current.coords[1]);
		ROS_INFO(" t_f = %.2f, t_g = %.2f",current.t_f,current.t_g);
		getchar();*/
		
		if(goal_coords[0]==-1){
			if(matrix[current.coords[0]][current.coords[1]]==val_to_find){
				ROS_INFO("Found solution at %d %d!",current.coords[0],current.coords[1]);
				return retrieve_path(current, &closedset); // later, the path
			}
		}else if(current.coords[0]==goal_coords[0] && current.coords[1]==goal_coords[1]){
			ROS_INFO("Found solution at %d %d!",current.coords[0],current.coords[1]);
			return retrieve_path(current, &closedset);
		}
		
		closedset.push_node(current);
		
		
		// FOR EACH NEIGHBOUR
		
		if(current.coords[0]-1>=0){
			
			coords[0]=current.coords[0]-1;
			coords[1]=current.coords[1];
			
			if(matrix[coords[0]][coords[1]] != black){
				t_h=heur(n.coords,goal_coords);
				t_g = current.t_g + 1;
				t_f = t_g + t_h;
				
				/*if(nodes_set.check_if_in_set(coords)){
					
					n = nodes_set.pop_requested(n.coords); // so I can keep track of the f cost
					
				}else{
					n=create_node(coords,t_f,t_g,current.coords);
					
				}*/
				
				if(openset.check_if_in_set(coords)){
					// SHOULD BE READ NODE
					n = openset.read_node(coords); // only doing this to get the f value, if present				
				}else{
					n = create_node(coords,t_f,t_g,current.coords);
				}
				
				if(!closedset.check_if_in_set(n.coords) || t_f < n.t_f ){
					n.t_g=t_g;
					n.t_f=t_f;
					n.came_from[0]=current.coords[0];
					n.came_from[1]=current.coords[1];
					if(!openset.check_if_in_set(n.coords))
						openset.push_node(n);
					else{ // update the value of the node in openset
						
						openset.pop_requested(n.coords);
						openset.push_node(n);
					}
					
					//nodes_set.push_node(n);
					
				}
			}
			
		}
		if(current.coords[0]+1<lateral_size){
			
			coords[0]=current.coords[0]+1;
			coords[1]=current.coords[1];
			
			if(matrix[coords[0]][coords[1]] != black){
				t_h=heur(n.coords,goal_coords);
				t_g = current.t_g + 1;
				t_f = t_g + t_h;
				
				/*if(nodes_set.check_if_in_set(coords)){
					
					n = nodes_set.pop_requested(n.coords); // so I can keep track of the f cost
					
				}else{
					n=create_node(coords,t_f,t_g,current.coords);
					
				}*/
				
				if(openset.check_if_in_set(coords)){
					// SHOULD BE READ NODE
					n = openset.read_node(coords); // only doing this to get the f value, if present
	
				}else{
					n = create_node(coords,t_f,t_g,current.coords);
				}
				
				if(!closedset.check_if_in_set(n.coords) || t_f < n.t_f ){
					n.t_g=t_g;
					n.t_f=t_f;
					n.came_from[0]=current.coords[0];
					n.came_from[1]=current.coords[1];
					if(!openset.check_if_in_set(n.coords))
						openset.push_node(n);
					else{ // update the value of the node in openset
						openset.pop_requested(n.coords);
						openset.push_node(n);
					}
					
					//nodes_set.push_node(n);
					
				}
			}
			
		}
		if(current.coords[1]-1>=0){
			
			coords[0]=current.coords[0];
			coords[1]=current.coords[1]-1;
			
			if(matrix[coords[0]][coords[1]] != black){
				t_h=heur(n.coords,goal_coords);
				t_g = current.t_g + 1;
				t_f = t_g + t_h;
				
				/*if(nodes_set.check_if_in_set(coords)){
					
					n = nodes_set.pop_requested(n.coords); // so I can keep track of the f cost
					
				}else{
					n=create_node(coords,t_f,t_g,current.coords);
					
				}*/
				
				if(openset.check_if_in_set(coords)){
					// SHOULD BE READ NODE
					n = openset.read_node(coords); // only doing this to get the f value, if present
				}else{
					n = create_node(coords,t_f,t_g,current.coords);
				}
				
				if(!closedset.check_if_in_set(n.coords) || t_f < n.t_f ){
					
					n.t_g=t_g;
					n.t_f=t_f;
					n.came_from[0]=current.coords[0];
					n.came_from[1]=current.coords[1];
					if(!openset.check_if_in_set(n.coords))
						openset.push_node(n);
					else{ // update the value of the node in openset
						openset.pop_requested(n.coords);
						openset.push_node(n);
					}
					
					//nodes_set.push_node(n);
					
				}
			}
			
		}
		if(current.coords[1]+1<lateral_size){
			
			coords[0]=current.coords[0];
			coords[1]=current.coords[1]+1;
			
			if(matrix[coords[0]][coords[1]] != black){
				t_h=heur(n.coords,goal_coords);
				t_g = current.t_g + 1;
				t_f = t_g + t_h;
				
				/*if(nodes_set.check_if_in_set(coords)){
					
					n = nodes_set.pop_requested(n.coords); // so I can keep track of the f cost
					
				}else{
					n=create_node(coords,t_f,t_g,current.coords);
					
				}*/
				if(openset.check_if_in_set(coords)){
					// SHOULD BE READ NODE
					n = openset.read_node(coords); // only doing this to get the f value, if present
				}else{
					n = create_node(coords,t_f,t_g,current.coords);
				}
	
				if(!closedset.check_if_in_set(n.coords) || t_f < n.t_f ){
					
					n.t_g=t_g;
					n.t_f=t_f;
					n.came_from[0]=current.coords[0];
					n.came_from[1]=current.coords[1];
					if(!openset.check_if_in_set(n.coords))
						openset.push_node(n);
					else{ // update the value of the node in openset
						openset.pop_requested(n.coords);
						openset.push_node(n);
					}
					
				//nodes_set.push_node(n);
					
				}
			}
			
		}
		
		
	}
	
	ROS_INFO("Did not find anything :(");
	return return_error;
	
	
	
}

void convert_to_commands(std::vector<node> sol, std::vector<int> *commands, std::vector<double> *vals){
	
	return;
	
}


bool planning_service(path_planner::path_srv::Request &req, path_planner::path_srv::Response &res){
	
	int size = req.map.info.width;
	std::vector<node> solution;
	std::vector<int> commands;
	std::vector<double> vals;
	
	Occupancy_Grid.resize(size,blue);
	
	for(int i=0; i < size; i++)
		Occupancy_Grid[i]=req.map.data[i];
	
	
	solution = find_closest(req.x,req.y,Occupancy_Grid,req.goal);
	
	convert_to_commands(solution,&commands,&vals);
	
	res.commands=commands;
	res.vals=vals;
	res.size=vals.size();
	
	
	return true;
	
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "path_planner");
	ros::NodeHandle n;
	ros::Time init_time=ros::Time::now();
	ros::ServiceServer path_service;
	std::vector<node> path;
	node node_ptr;
	int i=0;

	
	/*ros::ServiceServer map_sender = n.advertiseService("/mapping/ProcessedMap", provide_map);*/
	path_service = n.advertiseService("/path_planner/plan_trajectory",planning_service);
	
	
	ROS_INFO("Started the Path Planner Node");

	ros::Rate loop_rate(1);
	
	
	while(ros::ok()){

		//path=find_closest(7,2,Occupancy_Grid,2);
		loop_rate.sleep();
		ros::spinOnce();
		//ROS_INFO("Time: %.2f",ros::Time::now().toSec()-init_time.toSec());
		
		/*i=path.size()-1;
		while(i>=0){
			node_ptr = path[i];
			i--;
			ROS_INFO("(%d,%d)",node_ptr.coords[0],node_ptr.coords[1]);
			
		}*/
	}
	return 0;
}

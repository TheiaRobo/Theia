#include "ros/ros.h"
#include "std_msgs/String.h"
#include <stdio.h>
#include <cmath>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>
#include <signal.h>
#include <path_planner/path_srv.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>

#include <vector>
#include "sets.h"


const int black=100;
const int blue=75;
const int gray=50;
const int white=0;
const int FORWARD=1;
const int ROTATE=2;
int execute=1;

std::vector<signed char>  Occupancy_Grid;
std::vector<int> commands;
std::vector<double> params;

ros::Publisher path_pub;
ros::Publisher wall_pub;
ros::Publisher map_pub;

char heading_map = 'E';
const float PI = 3.1415926f;
double matrix_res=0;


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
		return 5.0*t_h;
	}else{ // no goal, no heuristic
		return 0.0;
	}


}

double g_cost(int coords[2], int prev_coords[2], int prev_from_coords[2]){

	if(prev_coords[0]-prev_from_coords[0]!=0){ // movement along x

		if(coords[0]-prev_coords[0]!=0){ // keep the same direction of movement

			return 1;

		}else{ // implies rotation

			return 500;
		}

	}else{ // movement along y

		if(coords[1]-prev_coords[1]!=0){

			return 1;
		}else{
			return 500;
		}
	}

	return 1; // just in case
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
	
	if(x_i < 0 || y_i < 0 || x_i >= lateral_size || y_i >= lateral_size){
		
		ROS_ERROR("Invalid initial coordinates");
		return return_error;
		
	}

	// values different from white and val_to_find are obstacles

	if(val_to_find==-1){ // back to start
		ROS_WARN("Will plan a path back to the start");

		goal_coords[0]=lateral_size/2;
		goal_coords[1]=lateral_size/2;

		ROS_WARN("My goal is set to the start position: (%d,%d)",goal_coords[0],goal_coords[1]);

	}else{
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
	}

	if(val_to_find==gray){ // inefficient, but better than running A* without an heuristic

		for(int i=0; i < lateral_size/2-1 && goal_coords[0]==NO_VAL;i++){
			for(int x=x_i-i/2; x < x_i+i/2;x++){
				for(int y=y_i-i/2; y<y_i+i/2;y++){

					if(matrix[x][y]==gray){
						goal_coords[0]=x;
						goal_coords[1]=y;
					}


				}
			}
		}

	}


	ROS_WARN("My goal is set to: (%d,%d)",goal_coords[0],goal_coords[1]);


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

	return_error.push_back(current);

	if(goal_coords[0]==NO_VAL){
		ROS_INFO("No solution exists");
		return return_error;
	}

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

		if(current.coords[0]==goal_coords[0] && current.coords[1]==goal_coords[1]){
			ROS_INFO("Found solution at %d %d!",current.coords[0],current.coords[1]);
			return retrieve_path(current, &closedset);
		}

		closedset.push_node(current);


		// FOR EACH NEIGHBOUR

		if(current.coords[0]-1>=0){

			coords[0]=current.coords[0]-1;
			coords[1]=current.coords[1];

			if(matrix[coords[0]][coords[1]] == white || matrix[coords[0]][coords[1]] == val_to_find){
				t_h=heur(n.coords,goal_coords);
				t_g = current.t_g + g_cost(coords,current.coords,current.came_from);
				t_f = t_g + t_h;

				if(openset.check_if_in_set(coords)){
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
				}
			}

		}
		if(current.coords[0]+1<lateral_size){

			coords[0]=current.coords[0]+1;
			coords[1]=current.coords[1];

			if(matrix[coords[0]][coords[1]] == white || matrix[coords[0]][coords[1]] == val_to_find){
				t_h=heur(n.coords,goal_coords);
				t_g = current.t_g + g_cost(coords,current.coords,current.came_from);
				t_f = t_g + t_h;

				if(openset.check_if_in_set(coords)){
					n = openset.read_node(coords); 

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
					else{ 
						openset.pop_requested(n.coords);
						openset.push_node(n);
					}

				}
			}

		}
		if(current.coords[1]-1>=0){

			coords[0]=current.coords[0];
			coords[1]=current.coords[1]-1;

			if(matrix[coords[0]][coords[1]] == white || matrix[coords[0]][coords[1]] == val_to_find){
				t_h=heur(n.coords,goal_coords);
				t_g = current.t_g + g_cost(coords,current.coords,current.came_from);
				t_f = t_g + t_h;

				if(openset.check_if_in_set(coords)){
					n = openset.read_node(coords); 
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
					else{ 
						openset.pop_requested(n.coords);
						openset.push_node(n);
					}

				}
			}

		}
		if(current.coords[1]+1<lateral_size){

			coords[0]=current.coords[0];
			coords[1]=current.coords[1]+1;

			if(matrix[coords[0]][coords[1]] == white || matrix[coords[0]][coords[1]] == val_to_find){
				t_h=heur(n.coords,goal_coords);
				t_g = current.t_g + g_cost(coords,current.coords,current.came_from);
				t_f = t_g + t_h;

				if(openset.check_if_in_set(coords)){
					n = openset.read_node(coords); 
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
					else{ 
						openset.pop_requested(n.coords);
						openset.push_node(n);
					}

				}
			}

		}


	}

	ROS_INFO("Did not find anything :(");
	return return_error;



}

void convert_to_commands(std::vector<node> sol, std::vector<int> *commands, std::vector<double> *vals){

	node current,prev;
	int forward_counter=0;



	for(int i=sol.size()-1; i > 0; i--){

		prev=sol[i];
		current=sol[i-1];

		switch(heading_map){
		case 'E':
			if(current.coords[0]-prev.coords[0] > 0){ // We are still moving East

				forward_counter++;

			}else{ // We rotated

				if(i!=sol.size()-1)
					forward_counter++;

				if(current.coords[0]-prev.coords[0] < 0){ // we went backwards

					(*commands).push_back(FORWARD);
					(*vals).push_back(forward_counter); // Respective parameter
					forward_counter=0;

					(*commands).push_back(ROTATE);
					(*vals).push_back(PI);
					heading_map = 'W';


				}else if(current.coords[1]-prev.coords[1] > 0){ // we rotated left

					(*commands).push_back(FORWARD);
					(*vals).push_back(forward_counter); // Respective parameter
					forward_counter=0;

					(*commands).push_back(ROTATE);
					(*vals).push_back(PI/2);

					heading_map = 'N';


				}else if(current.coords[1]-prev.coords[1] < 0){ // we rotated right

					(*commands).push_back(FORWARD);
					(*vals).push_back(forward_counter); // Respective parameter
					forward_counter=0;

					(*commands).push_back(ROTATE);
					(*vals).push_back(-PI/2);

					heading_map = 'S';

				}

			}
			break;
		case 'W':
			if(current.coords[0]-prev.coords[0] < 0){

				forward_counter++;

			}else{ 
				if(i!=sol.size()-1)
					forward_counter++;

				if(current.coords[0]-prev.coords[0] > 0){

					(*commands).push_back(FORWARD);
					(*vals).push_back(forward_counter); 
					forward_counter=0;

					(*commands).push_back(ROTATE);
					(*vals).push_back(PI);

					heading_map = 'E';


				}else if(current.coords[1]-prev.coords[1] > 0){ 

					(*commands).push_back(FORWARD);
					(*vals).push_back(forward_counter); 
					forward_counter=0;

					(*commands).push_back(ROTATE);
					(*vals).push_back(-PI/2);

					heading_map = 'N';


				}else if(current.coords[1]-prev.coords[1] < 0){

					(*commands).push_back(FORWARD);
					(*vals).push_back(forward_counter); 
					forward_counter=0;

					(*commands).push_back(ROTATE);
					(*vals).push_back(PI/2);

					heading_map = 'S';


				}

			}
			break;
		case 'S':
			if(current.coords[1]-prev.coords[1] < 0){

				forward_counter++;

			}else{ 
				if(i!=sol.size()-1)
					forward_counter++;

				if(current.coords[1]-prev.coords[1] > 0){

					(*commands).push_back(FORWARD);
					(*vals).push_back(forward_counter); 
					forward_counter=0;

					(*commands).push_back(ROTATE);
					(*vals).push_back(PI);

					heading_map = 'N';


				}else if(current.coords[0]-prev.coords[0] > 0){ 

					(*commands).push_back(FORWARD);
					(*vals).push_back(forward_counter); 
					forward_counter=0;

					(*commands).push_back(ROTATE);
					(*vals).push_back(PI/2);

					heading_map = 'E';


				}else if(current.coords[0]-prev.coords[0] < 0){

					(*commands).push_back(FORWARD);
					(*vals).push_back(forward_counter); 
					forward_counter=0;

					(*commands).push_back(ROTATE);
					(*vals).push_back(-PI/2);

					heading_map = 'W';


				}

			}
			break;
		case 'N':
			if(current.coords[1]-prev.coords[1] > 0){

				forward_counter++;

			}else{ 

				if(i!=sol.size()-1)
					forward_counter++;

				if(current.coords[1]-prev.coords[1] < 0){

					(*commands).push_back(FORWARD);
					(*vals).push_back(forward_counter); 
					forward_counter=0;

					(*commands).push_back(ROTATE);
					(*vals).push_back(PI);

					heading_map = 'S';


				}else if(current.coords[0]-prev.coords[0] > 0){ 

					(*commands).push_back(FORWARD);
					(*vals).push_back(forward_counter); 
					forward_counter=0;

					(*commands).push_back(ROTATE);
					(*vals).push_back(-PI/2);

					heading_map = 'E';


				}else if(current.coords[0]-prev.coords[0] < 0){

					(*commands).push_back(FORWARD);
					(*vals).push_back(forward_counter); 
					forward_counter=0;

					(*commands).push_back(ROTATE);
					(*vals).push_back(PI/2);

					heading_map = 'W';


				}

			}
			break;





		}		



	} // end for
	
	// at the end of the for cycle we may still have forward counter != 0. If that's not the case, we set the last rotation to zero for convenience
	
//	if(forward_counter!=0){ 
	(*commands).push_back(FORWARD);
	(*vals).push_back(forward_counter);
	//}
	/*else{
		int size = (*commands).size();
		
		if((*commands)[size-1] == 2){
		
			ROS_WARN("LAST COMMAND IS A ROTATION THAT I AM GOING TO ELIMINATE");
			(*vals)[size-1] = 0.0;
		
		}
		
	
	}*/
	
	
	return;

}


bool planning_service(path_planner::path_srv::Request &req, path_planner::path_srv::Response &res){



	int size = req.map.info.width;
	std::vector<node> solution;
	std::vector<int> commands;
	std::vector<double> vals;
	visualization_msgs::Marker object_marker;
	std::vector<geometry_msgs::Point> pos_list;
	geometry_msgs::Point pos;
	nav_msgs::OccupancyGrid Correct_Map_Msg;

	Occupancy_Grid.resize(size*size,blue);
	heading_map = req.heading;
	matrix_res = req.map.info.resolution;

	for(int i=0; i < size*size; i++)
		Occupancy_Grid[i]=req.map.data[i];


	Correct_Map_Msg.info=req.map.info;
	Correct_Map_Msg.data=Occupancy_Grid;

	map_pub.publish(Correct_Map_Msg);

	solution = find_closest(req.x,req.y,Occupancy_Grid,req.goal);

	convert_to_commands(solution,&commands,&vals);

	res.commands=commands;
	res.vals=vals;
	res.size=vals.size();

	object_marker.header.frame_id = "/mapping";
	object_marker.header.stamp = ros::Time::now();

	// Set the namespace and id for this marker.  This serves to create a unique ID
	// Any marker sent with the same namespace and id will overwrite the old one
	object_marker.ns = "path";
	object_marker.id = 0;

	object_marker.type = visualization_msgs::Marker::CUBE_LIST;

	// Set the marker action.  Options are ADD and DELETE
	object_marker.action = visualization_msgs::Marker::ADD;

	// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header


	object_marker.scale.x = matrix_res;
	object_marker.scale.y = matrix_res;
	object_marker.scale.z = matrix_res;

	// Set the color -- be sure to set alpha to something non-zero!
	object_marker.color.r = 0.0f;
	object_marker.color.g = 0.0f;
	object_marker.color.b = 1.0f;
	object_marker.color.a = 1.0;
	object_marker.pose.orientation.w=1.0;

	object_marker.lifetime = ros::Duration();


	for(int i=0; i<solution.size(); i++){
		pos.x=solution[i].coords[0]*0.01;
		pos.y=solution[i].coords[1]*0.01;
		pos.z=0;

		pos_list.push_back(pos);
	}


	object_marker.points=pos_list;
	
	for(int i = 0 ; i < vals.size(); i++){
	
		ROS_WARN("PLANNED PATH: (%d,%.2f)", commands[i], vals[i]);
	}
	
	
	path_pub.publish(object_marker);


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


	path_service = n.advertiseService("/path_planner/plan_trajectory",planning_service);
	path_pub = n.advertise<visualization_msgs::Marker>("/path_planner/trajectory",1);
	wall_pub = n.advertise<visualization_msgs::Marker>("/path_planner/walls",1);
	map_pub = n.advertise<nav_msgs::OccupancyGrid>("/path_planner/map",1);

	ROS_INFO("Started the Path Planner Node");

	ros::Rate loop_rate(1);


	while(ros::ok()){
		loop_rate.sleep();
		ros::spinOnce();

	}
	return 0;
}

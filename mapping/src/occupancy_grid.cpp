#include "ros/ros.h"
#include "std_msgs/String.h"
#include <stdio.h>
#include <cmath>
#include <core_control_motor/vw.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>
#include <control_logic/info.h>
#include <core_sensors/ir.h>
#include <vector>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>

 //Initialize
const float PI=3.1415926f;
const double resolution_matrix=0.01; // in meter!
const int x_matrix=5/resolution_matrix; 
const int y_matrix=5/resolution_matrix;

// robot size: 2*delta_x+1, 2*delta_y+1
const int robot_delta_x=10; // robot is a dot
const int robot_delta_y=10;


// sensor array

double ir[8];
double inf_thres=20;

int x_Current_Pose=round(x_matrix/2); //x,y coordinates
int y_Current_Pose=round(y_matrix/2); //x,y coordinates
const int freq=100;


// high level info

int wall=-1;
char heading='E';


std::vector<signed char>  Occupancy_Grid(x_matrix*y_matrix,50);
ros::Publisher occ_pub;
ros::Publisher map_pub;
ros::Publisher robo_pub;
ros::Publisher odo_pub;
ros::Subscriber	odometry_sub;
ros::Subscriber	camera_sub;
ros::Subscriber	logic_sub;
ros::Subscriber ir_sub;


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
	double y_Pose_Odometry=-cell_round(odometry_msg->pose.pose.position.y*100);
	
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

void ir_line(int ir_num,int x_position, int y_position,double ir_val){
	
	int startx=x_position, starty=y_position;
	
	switch(heading){
	
	case 'E':
		switch(ir_num){
		
		case 1:
		case 3:
			startx+=robot_delta_x;
			starty-=robot_delta_y;
			break;
		case 2:
		case 5:
			startx+=robot_delta_x;
			starty+=robot_delta_y;
			break;
		case 4:
			startx-=robot_delta_x;
			starty-=robot_delta_y;
			break;
		case 6:
			startx-=robot_delta_x;
			starty+=robot_delta_y;
			break;
		}
		break;
	case 'W':
		switch(ir_num){
		
		case 1:
		case 3:
			startx-=robot_delta_x;
			starty+=robot_delta_y;
			break;
		case 2:
		case 5:
			startx-=robot_delta_x;
			starty-=robot_delta_y;
			break;
		case 4:
			startx+=robot_delta_x;
			starty+=robot_delta_y;
			break;
		case 6:
			startx+=robot_delta_x;
			starty-=robot_delta_y;
			break;		
		}
		break;
	case 'N':
		switch(ir_num){

		case 1:
		case 3:
			startx-=robot_delta_x;
			starty-=robot_delta_y;
			break;
		case 5:
		case 2:
			startx+=robot_delta_x;
			starty-=robot_delta_y;
			break;
		case 4:
			startx-=robot_delta_x;
			starty+=robot_delta_y;
			break;
		case 6:
			startx+=robot_delta_x;
			starty+=robot_delta_y;

		}
		break;
	case 'S':
		switch(ir_num){

		case 1:
		case 3:
			startx+=robot_delta_x;
			starty+=robot_delta_y;
			break;
		case 5:
		case 2:
			startx-=robot_delta_x;
			starty+=robot_delta_y;
			break;
		case 4:
			startx+=robot_delta_x;
			starty-=robot_delta_y;
			break;
		case 6:
			startx-=robot_delta_x;
			starty-=robot_delta_y;
		}
		break;

	}
	
	switch(heading){
	
	case 'E':
		switch(ir_num){
		
		case 1:
		case 2:
			for(int x=startx; x<startx+cell_round(ir_val);x++){
				Occupancy_Grid[x*x_matrix+starty]=0;
			}
			if(ir_val!=inf_thres)
				Occupancy_Grid[(startx+cell_round(ir_val))*x_matrix+starty]=100;
			break;
		case 3:
		case 4:
			for(int y=starty; y>starty-cell_round(ir_val);y--){
				Occupancy_Grid[startx*x_matrix+y]=0;
			}
			if(ir_val!=inf_thres)
				Occupancy_Grid[startx*x_matrix+starty-cell_round(ir_val)]=100;
			break;
		case 5:
		case 6:
			for(int y=starty; y<starty+cell_round(ir_val);y++){
				Occupancy_Grid[startx*x_matrix+y]=0;
			}
			if(ir_val!=inf_thres)
				Occupancy_Grid[startx*x_matrix+starty+cell_round(ir_val)]=100;
			break;
			
		}
		break;
	case 'W':
		switch(ir_num){

		case 1:
		case 2:
			for(int x=startx; x>startx-cell_round(ir_val);x--){
				Occupancy_Grid[x*x_matrix+starty]=0;
			}
			if(ir_val!=inf_thres)
				Occupancy_Grid[(startx-cell_round(ir_val))*x_matrix+starty]=100;
			break;
		case 3:
		case 4:
			for(int y=starty; y<starty+cell_round(ir_val);y++){
				Occupancy_Grid[startx*x_matrix+y]=0;
			}
			if(ir_val!=inf_thres)
				Occupancy_Grid[startx*x_matrix+starty+cell_round(ir_val)]=100;
			break;
		case 5:
		case 6:
			for(int y=starty; y>starty-cell_round(ir_val);y--){
				Occupancy_Grid[startx*x_matrix+y]=0;
			}
			if(ir_val!=inf_thres)
				Occupancy_Grid[startx*x_matrix+starty-cell_round(ir_val)]=100;
			break;
			
		}
		break;
	case 'N':
		switch(ir_num){

		case 1:
		case 2:
			for(int y=starty; y>starty-cell_round(ir_val);y--){
				Occupancy_Grid[startx*x_matrix+y]=0;
			}
			if(ir_val!=inf_thres)
				Occupancy_Grid[startx*x_matrix+starty-cell_round(ir_val)]=100;
			break;
		case 3:
		case 4:
			for(int x=startx; x>startx-cell_round(ir_val);x--){
				Occupancy_Grid[x*x_matrix+starty]=0;
			}
			if(ir_val!=inf_thres)
				Occupancy_Grid[(startx-cell_round(ir_val))*x_matrix+starty]=100;
			break;
		case 5:
		case 6:
			for(int x=startx; x<startx+cell_round(ir_val);x++){
				Occupancy_Grid[x*x_matrix+starty]=0;
			}
			if(ir_val!=inf_thres)
				Occupancy_Grid[(startx+cell_round(ir_val))*x_matrix+starty]=100;
			break;
			
		}
		break;
	case 'S':
		switch(ir_num){

		case 1:
		case 2:
			for(int y=starty; y<starty+cell_round(ir_val);y++){
				Occupancy_Grid[startx*x_matrix+y]=0;
			}
			if(ir_val!=inf_thres)
				Occupancy_Grid[startx*x_matrix+starty+cell_round(ir_val)]=100;
			break;
		case 3:
		case 4:
			for(int x=startx; x<startx+cell_round(ir_val);x++){
				Occupancy_Grid[x*x_matrix+starty]=0;
			}
			if(ir_val!=inf_thres)
				Occupancy_Grid[(startx+cell_round(ir_val))*x_matrix+starty]=100;
			break;
		case 5:
		case 6:
			for(int x=startx; x>startx-cell_round(ir_val);x--){
				Occupancy_Grid[x*x_matrix+starty]=0;
			}
			if(ir_val!=inf_thres)
				Occupancy_Grid[(startx-cell_round(ir_val))*x_matrix+starty]=100;
			break;
			
		}
		break;
	
	
	}
	
	
	
}

void place_ir(){
	
	if(wall!=-1){
		for(int i=0; i<6; i++){
			if(ir[i]<inf_thres)
				ir_line(i+1,x_Current_Pose, y_Current_Pose,ir[i]);
			else
				ir_line(i+1,x_Current_Pose,y_Current_Pose,inf_thres);
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

void Get_Ir(core_sensors::ir::ConstPtr ir_msg){
	
	for(int i=0; i<8; i++)
		ir[i]=ir_msg->dist[i];
	
}

void Get_Motion_Info(control_logic::info::ConstPtr logic_msg) {
	
	wall=logic_msg->info_wall;
	heading=logic_msg->info_heading;
	
	ROS_INFO("Heading: %c Wall: %d",heading,wall);
	
}

void update_robot(){
	
	visualization_msgs::Marker marker;
	visualization_msgs::Marker odo;
	// Set the frame ID and timestamp.  See the TF tutorials for information on these.
	marker.header.frame_id = "/mapping";
	marker.header.stamp = ros::Time::now();

	// Set the namespace and id for this marker.  This serves to create a unique ID
	// Any marker sent with the same namespace and id will overwrite the old one
	marker.ns = "robot_pos";
	marker.id = 0;

	marker.type = visualization_msgs::Marker::CUBE;

	// Set the marker action.  Options are ADD and DELETE
	marker.action = visualization_msgs::Marker::ADD;

	// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
	marker.pose.position.x = x_Current_Pose;
	marker.pose.position.y = y_Current_Pose;
	marker.pose.position.z = 0;
	
	switch(heading){
	case 'E':
		marker.pose.orientation = tf::createQuaternionMsgFromYaw(0);
		break;
	case 'S':
		marker.pose.orientation = tf::createQuaternionMsgFromYaw(-PI/2);
		break;
	case 'W':
		marker.pose.orientation = tf::createQuaternionMsgFromYaw(PI);
		break;
	case 'N':
		marker.pose.orientation = tf::createQuaternionMsgFromYaw(PI/2);
		break;
	}
	
	

	// Set the scale of the marker -- 1x1x1 here means 1m on a side
	marker.scale.x = 1.2;
	marker.scale.y = 1.2;
	marker.scale.z = 1.2;

	// Set the color -- be sure to set alpha to something non-zero!
	marker.color.r = 0.0f;
	marker.color.g = 1.0f;
	marker.color.b = 0.0f;
	marker.color.a = 1.0;

	marker.lifetime = ros::Duration();
	
	odo=marker;
	
	odo.type = visualization_msgs::Marker::ARROW;
	odo.pose.position.z=0.2;
	odo.id = 1;

	// Publish the marker
	robo_pub.publish(marker);
	odo_pub.publish(odo);

}


// ########################## MAIN ################################

int main(int argc, char **argv)
{
	ros::init(argc, argv, "mapping");
	ros::NodeHandle n;

    
	odometry_sub = n.subscribe("/core_sensors_odometry/odometry",1,Get_Readings_Odometry);
	logic_sub = n.subscribe("/control_logic/info",1,Get_Motion_Info);
	ir_sub = n.subscribe("/core_sensors_ir/ir",1,Get_Ir);
	
	occ_pub = n.advertise<nav_msgs::OccupancyGrid>("/mapping/occ",1);
	map_pub = n.advertise<nav_msgs::MapMetaData>("/mapping/map",1);
	robo_pub = n.advertise<visualization_msgs::Marker>("/mapping/robot", 1);
	odo_pub = n.advertise<visualization_msgs::Marker>("/mapping/odo", 1);


	ROS_INFO("Started the mapping Node");

	ros::Rate loop_rate(freq);

	struct timeval start, end;
	
	for(int i=0; i<8; i++)
		ir[i]=0.0;
	
	while(ros::ok()){

		place_map(x_Current_Pose,y_Current_Pose,robot_delta_x,robot_delta_y,0);
		place_ir();
		update_robot();
		Send_Message();

		loop_rate.sleep();
		ros::spinOnce();


	}
	return 0;
}

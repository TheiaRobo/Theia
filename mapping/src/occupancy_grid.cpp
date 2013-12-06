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
#include <string>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include <theia_services/mapsrv.h>
#include <theia_services/corrected_odo.h>
#include <vision_object/Object.h>

 //Initialize
const float PI=3.1415926f;
const double resolution_matrix=0.01; // in meter!
const int x_matrix=15/resolution_matrix; 
const int y_matrix=15/resolution_matrix;

typedef struct _object{
	std::string name;
	int num;
}object;

std::vector<object> object_list;
std::vector<geometry_msgs::Point> pos_list;


// robot size: 2*delta_x+1, 2*delta_y+1
const int robot_delta_x=10; // robot is a dot
const int robot_delta_y=10;

const int black=100;
const int blue=75;
const int gray=50;
const int white=0;


// sensor array

double ir[8];
double inf_thres=20;


int x_Current_Pose=round(x_matrix/2); //x,y coordinates
int y_Current_Pose=round(y_matrix/2); //x,y coordinates


// variables needed to perform odometry's correction
// raw odometry
double odo_x[2]={x_matrix*resolution_matrix/2,x_matrix*resolution_matrix/2};
double odo_y[2]={y_matrix*resolution_matrix/2,y_matrix*resolution_matrix/2};
double odo_theta=0.0;
double camera_initial_z=0;
double Lat_0 = 22.5*PI/180;
double Long_0 = 0;

// corrected odometry
double corrected_odo_x[2]={x_matrix*resolution_matrix/2,x_matrix*resolution_matrix/2};
double corrected_odo_y[2]={y_matrix*resolution_matrix/2,y_matrix*resolution_matrix/2};
double odo_correct=0.0;
double s_delta_x=0.0;
double s_delta_y=0.0;

const int freq=100;


// high level info

int wall=-1;
char heading='E';


std::vector<signed char>  Occupancy_Grid(x_matrix*y_matrix,blue);
std::vector<signed char>  Corrected_Map(x_matrix*y_matrix,blue);
ros::Publisher occ_pub;
ros::Publisher map_pub;
ros::Publisher robo_pub;
ros::Publisher arrow_pub;
ros::Publisher object_pub;
ros::Publisher odo_pub;
ros::Publisher corrected_map_pub;
ros::Publisher talk_pub;
ros::Subscriber	odometry_sub;
ros::Subscriber	camera_sub;
ros::Subscriber	logic_sub;
ros::Subscriber ir_sub;


/* cell_round
 * Rounds the received value to the nearest cell coordinate
 */
int cell_round(double val){
	double cm_res=resolution_matrix*100;
	
	return round(val/cm_res);
	
}


void Get_Readings_Odometry(nav_msgs::Odometry::ConstPtr odometry_msg){

	//double x_Pose_Odometry=odometry_msg->pose.pose.position.x*100; //odometry is comming in meters
	//double y_Pose_Odometry=odometry_msg->pose.pose.position.y*100;
	double delta_x=0.0, delta_y=0.0;	
	
	odo_x[1]=odo_x[0];
	odo_y[1]=odo_y[0];
	corrected_odo_x[1]=corrected_odo_x[0];
	corrected_odo_y[1]=corrected_odo_y[0];
	
	odo_x[0]=odometry_msg->pose.pose.position.x+x_matrix*resolution_matrix/2;
	odo_y[0]=odometry_msg->pose.pose.position.y+y_matrix*resolution_matrix/2;
	odo_theta=atan2(odometry_msg->pose.pose.orientation.z,odometry_msg->pose.pose.orientation.w)*2;
	
	delta_x=odo_x[0]-odo_x[1];
	delta_y=odo_y[0]-odo_y[1];
	
	//ROS_INFO("x_Current_Pose: %d\ny_Current_Pose: %d",x_Current_Pose,y_Current_Pose);
	
	//ROS_INFO("ODO_THETA: %.2f",odo_theta);
	//ROS_INFO("Delta_x: %.2f\n Delta_y: %.2f",delta_x,delta_y);
	switch(heading){
	case 'E':
		odo_correct=(odo_theta-0);
		delta_x=sqrt(delta_x*delta_x+delta_y*delta_y)*cos(odo_correct);
		delta_y=0;		
		break;
	case 'W':
		odo_correct=(2*PI-odo_theta);
		delta_x=sqrt(delta_x*delta_x+delta_y*delta_y)*cos(odo_correct);
		delta_y=0;
		break;
	case 'N':
		odo_correct=(PI-odo_theta);
		delta_y=sqrt(delta_x*delta_x+delta_y*delta_y)*sin(odo_correct);
		delta_x=0;
		break;
	case 'S':
		odo_correct=(-PI-odo_theta);
		delta_y=sqrt(delta_x*delta_x+delta_y*delta_y)*sin(odo_correct);
		delta_x=0;
		break;
	}
	//ROS_INFO("ODO_CORRECT: %.2f\nDelta_x: %.2f\n Delta_y: %.2f",odo_correct,delta_x,delta_y);
	corrected_odo_x[0]=corrected_odo_x[1]+delta_x;
	corrected_odo_y[0]=corrected_odo_y[1]+delta_y;
	
	//s_delta_x=cell_round((corrected_odo_x[0]-corrected_odo_x[1])*100);
	//s_delta_y=cell_round((corrected_odo_y[0]-corrected_odo_y[1])*100);
	
	x_Current_Pose=cell_round(corrected_odo_x[0]*100);
	y_Current_Pose=cell_round(corrected_odo_y[0]*100);
	
	//ROS_INFO("x_Current_Pose: %d\ny_Current_Pose: %d",x_Current_Pose,y_Current_Pose);
	
}


std::vector<signed char> place_map(std::vector<signed char> map,int x_position, int y_position, int delta_x, int delta_y, int val) {
	
	if(val!=white && val!=black && val!=gray && val!=blue)
		ROS_INFO("Will place object num %d",val);
	
	

	for (int x=x_position-delta_x;x<=x_position+delta_x;x++){
		for (int y=(y_position-delta_y)*y_matrix;y<=(y_position+delta_y)*y_matrix;y+=y_matrix){
		
			if(map[x+y]==white || map[x+y]==black || map[x+y]==gray || map[x+y] == blue)
				map[x+y]=val;

		}
	}
	
	return map;
	
}

void Correct_Map(int x_pixel_delta,int y_pixel_delta){

	Corrected_Map=Occupancy_Grid;
	for(int x=0; x < x_matrix; x++){
		for(int y=0; y < y_matrix*y_matrix; y+=y_matrix){
			//ROS_INFO("x: %d y:%d",x,y);
			//Occupancy_Grid[x+y]=1;
			if(Occupancy_Grid[x+y]==black)
				Corrected_Map=place_map(Corrected_Map,x,y/y_matrix,robot_delta_x,robot_delta_y,black);
		}
	}
	

}

/* Fill line
 * Creates a line on the grid starting on (startx,starty) and moving along the given axis n cells
 * 
 *  n: size of the line
 * axis: 1:x, 2:y, -1:-x, -2:-y
 * 
 */
void fill_line(int startx, int starty, int axis,int n,int val){
	
	int out=0;
	// if val == 100 or val == 50 we can go over the line and place 75 or 50, respectively 
	
	switch(axis){
	
	case 1:
		for(int x=startx; x<startx+n;x++){
			if(Occupancy_Grid[x+starty*y_matrix]==white || Occupancy_Grid[x+starty*y_matrix]==black || Occupancy_Grid[x+starty*y_matrix] == blue || Occupancy_Grid[x+starty*y_matrix] == gray)
				Occupancy_Grid[x+starty*y_matrix]=val;
		}
		
		if(val == white){
		
			for(int x = startx + n + 1; x < x_matrix && Occupancy_Grid[x+starty*y_matrix] == blue; x++){
				
				Occupancy_Grid[x+starty*y_matrix] = gray;
			}
		
		}
		
		
		
		break;
	case 2:
		for(int y=starty; y<starty+n;y++){
			if(Occupancy_Grid[startx+y*y_matrix]==white || Occupancy_Grid[startx+y*y_matrix]==black || Occupancy_Grid[startx+y*y_matrix] == blue || Occupancy_Grid[startx+y*y_matrix] == gray)
				Occupancy_Grid[startx+y*y_matrix]=val;
		}
		
		
		if(val == white){
		
			for(int y = starty + n + 1; y < y_matrix && Occupancy_Grid[startx+y*y_matrix] == blue; y++){
				
				Occupancy_Grid[startx+y*y_matrix] = gray;
			}
		
		}
		
		
		
		break;
	case -1:
		for(int x=startx; x>startx-n;x--){
			if(Occupancy_Grid[x+starty*y_matrix]==white || Occupancy_Grid[x+starty*y_matrix]==black || Occupancy_Grid[x+starty*y_matrix] == blue || Occupancy_Grid[x+starty*y_matrix] == gray)
				Occupancy_Grid[x+starty*y_matrix]=val;
		}
		
		if(val == white){
		
			for(int x = startx - n - 1; x >= 0 && Occupancy_Grid[x+starty*y_matrix] == blue; x--){
				
				Occupancy_Grid[x+starty*y_matrix] = gray;
			}
		
		}
		
		
		
		break;
	case -2:
		for(int y=starty; y>starty-n;y--){
			if(Occupancy_Grid[startx+y*y_matrix]==white || Occupancy_Grid[startx+y*y_matrix]==black || Occupancy_Grid[startx+y*y_matrix] == blue || Occupancy_Grid[startx+y*y_matrix] == gray)
				Occupancy_Grid[startx+y*y_matrix]=val;
		}
		
		if(val == white){
		
			for(int y = starty - n - 1; y > 0 && Occupancy_Grid[startx+y*y_matrix] == blue; y--){
				
				Occupancy_Grid[startx+y*y_matrix] = gray;
			}
		
		}
		
		break;
	
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
			starty+=robot_delta_y;
			break;
		case 2:
		case 5:
			startx+=robot_delta_x;
			starty-=robot_delta_y;
			break;
		case 4:
			startx-=robot_delta_x;
			starty+=robot_delta_y;
			break;
		case 6:
			startx-=robot_delta_x;
			starty-=robot_delta_y;
			break;
		}
		break;
	case 'W':
		switch(ir_num){
		
		case 1:
		case 3:
			startx-=robot_delta_x;
			starty-=robot_delta_y;
			break;
		case 2:
		case 5:
			startx-=robot_delta_x;
			starty+=robot_delta_y;
			break;
		case 4:
			startx+=robot_delta_x;
			starty-=robot_delta_y;
			break;
		case 6:
			startx+=robot_delta_x;
			starty+=robot_delta_y;
			break;		
		}
		break;
	case 'N':
		switch(ir_num){

		case 1:
		case 3:
			startx-=robot_delta_x;
			starty+=robot_delta_y;
			break;
		case 5:
		case 2:
			startx+=robot_delta_x;
			starty+=robot_delta_y;
			break;
		case 4:
			startx-=robot_delta_x;
			starty-=robot_delta_y;
			break;
		case 6:
			startx+=robot_delta_x;
			starty-=robot_delta_y;

		}
		break;
	case 'S':
		switch(ir_num){

		case 1:
		case 3:
			startx+=robot_delta_x;
			starty-=robot_delta_y;
			break;
		case 5:
		case 2:
			startx-=robot_delta_x;
			starty-=robot_delta_y;
			break;
		case 4:
			startx+=robot_delta_x;
			starty+=robot_delta_y;
			break;
		case 6:
			startx-=robot_delta_x;
			starty+=robot_delta_y;
		}
		break;

	}
	
	switch(heading){
	
	case 'E':
		switch(ir_num){
		
		case 1:
		case 2:
			fill_line(startx,starty,1,cell_round(ir_val)-s_delta_x,0);
			if(ir_val!=inf_thres)
				fill_line(startx+cell_round(ir_val),starty,1,1,100);//Occupancy_Grid[startx+cell_round(ir_val)-s_delta_x+starty*y_matrix]=100;
			break;
		case 3:
		case 4:
			fill_line(startx,starty,2,cell_round(ir_val)-s_delta_y,0);
			if(ir_val!=inf_thres)
				fill_line(startx,(starty+cell_round(ir_val)),2,1,100);//Occupancy_Grid[startx+(starty+cell_round(ir_val)-s_delta_y)*y_matrix]=100;
			break;
		case 5:
		case 6:
			fill_line(startx,starty,-2,cell_round(ir_val)+s_delta_y,0);
			if(ir_val!=inf_thres)
				fill_line(startx,(starty-cell_round(ir_val)),-2,1,100);//Occupancy_Grid[startx+(starty-cell_round(ir_val)+s_delta_y)*y_matrix]=100;
			break;
			
		}
		break;
	case 'W':
		switch(ir_num){

		case 1:
		case 2:
			fill_line(startx,starty,-1,cell_round(ir_val)+s_delta_x,0);
			if(ir_val!=inf_thres)
				fill_line(startx-cell_round(ir_val),starty,-1,1,100);//Occupancy_Grid[startx-cell_round(ir_val)+s_delta_x+starty*y_matrix]=100;
			break;
		case 3:
		case 4:
			fill_line(startx,starty,-2,cell_round(ir_val)+s_delta_y,0);
			if(ir_val!=inf_thres)
				fill_line(startx,(starty-cell_round(ir_val)),-2,1,100);//Occupancy_Grid[startx+(starty-cell_round(ir_val)+s_delta_y)*y_matrix]=100;
			break;
		case 5:
		case 6:
			fill_line(startx,starty,2,cell_round(ir_val)-s_delta_y,0);
			if(ir_val!=inf_thres)
				fill_line(startx,(starty+cell_round(ir_val)),2,1,100);//Occupancy_Grid[startx+(starty+cell_round(ir_val)-s_delta_y)*y_matrix]=100;
			break;
			
		}
		break;
	case 'N':
		switch(ir_num){

		case 1:
		case 2:
			fill_line(startx,starty,2,cell_round(ir_val)-s_delta_y,0);
			if(ir_val!=inf_thres)
				fill_line(startx,(starty+cell_round(ir_val)),2,1,100);//Occupancy_Grid[startx+(starty+cell_round(ir_val)-s_delta_y)*y_matrix]=100;
			break;
		case 3:
		case 4:
			fill_line(startx,starty,-1,cell_round(ir_val)+s_delta_x,0);
			if(ir_val!=inf_thres)
				fill_line(startx-cell_round(ir_val),starty,-1,1,100);//Occupancy_Grid[startx-cell_round(ir_val)+s_delta_x+starty*y_matrix]=100;
			break;
		case 5:
		case 6:
			fill_line(startx,starty,1,cell_round(ir_val)-s_delta_x,0);
			if(ir_val!=inf_thres)
				fill_line(startx+cell_round(ir_val),starty,1,1,100);//Occupancy_Grid[startx+cell_round(ir_val)-s_delta_x+starty*y_matrix]=100;
			break;
			
		}
		break;
	case 'S':
		switch(ir_num){

		case 1:
		case 2:
			fill_line(startx,starty,-2,cell_round(ir_val)+s_delta_y,0);
			if(ir_val!=inf_thres)
				fill_line(startx,(starty-cell_round(ir_val)),-2,1,100);//Occupancy_Grid[startx+(starty-cell_round(ir_val)+s_delta_y)*y_matrix]=100;
			break;
		case 3:
		case 4:
			fill_line(startx,starty,1,cell_round(ir_val)-s_delta_x,0);
			if(ir_val!=inf_thres)
				fill_line(startx+cell_round(ir_val),starty,1,1,100);//Occupancy_Grid[startx+cell_round(ir_val)-s_delta_x+starty*y_matrix]=100;
			break;
		case 5:
		case 6:
			fill_line(startx,starty,-1,cell_round(ir_val)+s_delta_x,0);
			if(ir_val!=inf_thres)
				fill_line(startx-cell_round(ir_val),starty,-1,1,100);//Occupancy_Grid[startx-cell_round(ir_val)+s_delta_x+starty*y_matrix]=100;
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
	nav_msgs::OccupancyGrid Correct_Map_Msg;
	nav_msgs::MapMetaData map_msg;

	occ_msg.header.stamp=ros::Time::now();
	occ_msg.header.frame_id= "/mapping";

	map_msg.map_load_time=ros::Time::now();
	map_msg.resolution=resolution_matrix;
	map_msg.width=x_matrix;
	map_msg.height=y_matrix;
	
	occ_msg.info=map_msg;
	Correct_Map_Msg=occ_msg;
	

	occ_msg.data=Occupancy_Grid;
	Correct_Map_Msg.data=Corrected_Map;
	
	occ_pub.publish(occ_msg);
	corrected_map_pub.publish(Correct_Map_Msg);
}

/*
EVERZTHING IN METERZ
*/
double * convert_object_distance(double distX, double distY){
	//ROS_INFO("Lat %.5f", Lat);
	//ROS_INFO("Long %.5f", Long);
	//ROS_INFO("Dist: %.5f", dist);
	
	double * ret_val;
	//double x_0=0.0;
	//double y_0=0.0;
	//double z_0=camera_initial_z; //cms already
	//double x_i=0.0, y_i=0.0, z_i=0.35;
	
	// insert code to convert to (dx,dy)	
	//x_i=(y_i-y_0)*tan(Lat?Lat_0)+x_0;
	// x_i = 0.35 * tan(Lat + Lat_0) + x_0;
	//x_i = dist * sin(Lat + Lat_0) + x_0;
	//x_i=(z_i-(z_0))*tan(Lat+Lat_0)+x_0;
	//z_i=(x_i-x_0)*tan(Long-Long_0)+z_0;
	//y_i=((x_i-x_0)*tan(Long+Long_0)+(-y_0));
	//y_i = -(x_i - x_0) * tan(Long+Long_0);
	//y_i=y_0;
	//z_i=z_0;
	
	//Testing coord
	//x_i = dist * sin(Lat_0) + x_0;
	//y_i = 0;
		
	ret_val=new double(2);
	
	ret_val[0]= distX;
	ret_val[1]= distY;
	
	return ret_val;
	
}

void Place_Object(vision_object::Object::ConstPtr msg) {
	
	object new_object;
	visualization_msgs::Marker object_marker;
	double pos_x,pos_y,*distance=0;
	geometry_msgs::Point pos;
	std::string new_name=msg->objectName;
	std_msgs::String talk_msg;
	
	for(int i=0; i<object_list.size();i++){ // IMPROVE LATER
		if(object_list[i].name==new_name)
			return;
	}
	
	if(wall==-1)
		return;
	
	
	new_object.name=msg->objectName;
	new_object.num=object_list.size()+1;	
	object_list.push_back(new_object);
	

	distance=convert_object_distance(0.20,0);//msg->distX,msg->distY);	

	switch(heading){
		
		case 'E':
			pos_x=corrected_odo_x[0]+distance[0];
			pos_y=corrected_odo_y[0]+distance[1];
			
			break;
		case 'W':
			pos_x=corrected_odo_x[0]-distance[0];
			pos_y=corrected_odo_y[0]-distance[1];
			break;
		case 'N':
			pos_x=corrected_odo_x[0]-distance[1];
			pos_y=corrected_odo_y[0]+distance[0];
			break;
		case 'S':
			pos_x=corrected_odo_x[0]+distance[1];
			pos_y=corrected_odo_y[0]-distance[0];
			break;
	
	}
	
	Occupancy_Grid=place_map(Occupancy_Grid,cell_round(pos_x*100),cell_round(pos_y*100),robot_delta_x,robot_delta_y,new_object.num);
	
	ROS_INFO("New object: %s Num: %d at (%d,%d)",new_object.name.c_str(),new_object.num,cell_round(pos_x*100),cell_round(pos_y*100));
	
	talk_msg.data = new_object.name;
	talk_pub.publish(talk_msg);
	//robot/talk
	
	object_marker.header.frame_id = "/mapping";
	object_marker.header.stamp = ros::Time::now();

	// Set the namespace and id for this marker.  This serves to create a unique ID
	// Any marker sent with the same namespace and id will overwrite the old one
	object_marker.ns = "object_pos";
	object_marker.id = 0;

	object_marker.type = visualization_msgs::Marker::CUBE_LIST;

	// Set the marker action.  Options are ADD and DELETE
	object_marker.action = visualization_msgs::Marker::ADD;

	// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header

	pos.x=pos_x;
	pos.y=pos_y;
	pos.z=0;

	pos_list.push_back(pos);
	
		
	object_marker.points=pos_list;
	
	object_marker.scale.x = 0.2;
	object_marker.scale.y = 0.2;
	object_marker.scale.z = 0.2;

	// Set the color -- be sure to set alpha to something non-zero!
	object_marker.color.r = 1.0f;
	object_marker.color.g = 0.0f;
	object_marker.color.b = 0.0f;
	object_marker.color.a = 1.0;

	object_marker.lifetime = ros::Duration();
	
	object_pub.publish(object_marker);
	
	if(distance!=0)
		delete distance;
}

void Get_Ir(core_sensors::ir::ConstPtr ir_msg){
	
	for(int i=0; i<8; i++)
		ir[i]=ir_msg->dist[i];
	
}

void Get_Motion_Info(control_logic::info::ConstPtr logic_msg) {
	
	wall=logic_msg->info_wall;
	heading=logic_msg->info_heading;
	
	//ROS_INFO("Heading: %c Wall: %d",heading,wall);
	
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
	marker.pose.position.x = corrected_odo_x[0];
	marker.pose.position.y = corrected_odo_y[0];
	marker.pose.position.z = 0;
	
	switch(heading){
	case 'E':
		marker.pose.orientation = tf::createQuaternionMsgFromYaw(0);
		break;
	case 'S':
		marker.pose.orientation = tf::createQuaternionMsgFromYaw(-PI/2);
		break;
	case 'W':
		marker.pose.orientation = tf::createQuaternionMsgFromYaw(-PI);
		break;
	case 'N':
		marker.pose.orientation = tf::createQuaternionMsgFromYaw(PI/2);
		break;
	}
	
	

	// Set the scale of the marker -- 1x1x1 here means 1m on a side
	marker.scale.x = 0.2;
	marker.scale.y = 0.2;
	marker.scale.z = 0.2;

	// Set the color -- be sure to set alpha to something non-zero!
	marker.color.r = 0.0f;
	marker.color.g = 1.0f;
	marker.color.b = 0.0f;
	marker.color.a = 1.0;

	marker.lifetime = ros::Duration();
	
	odo=marker;
	
	odo.type = visualization_msgs::Marker::ARROW;
	odo.pose.position.z=0.1;
	odo.id = 1;

	// Publish the marker
	robo_pub.publish(marker);
	arrow_pub.publish(odo);

}

void Send_Odometry(){
	
	theia_services::corrected_odo msg;
	
	msg.x=corrected_odo_x[0];
	msg.y=corrected_odo_y[0];
	
	odo_pub.publish(msg);
	
}

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
	
}

// ########################## MAIN ################################

int main(int argc, char **argv)
{
	ros::init(argc, argv, "mapping");
	ros::NodeHandle n;

    
	odometry_sub = n.subscribe("/core_sensors_odometry/odometry",100000,Get_Readings_Odometry);
	logic_sub = n.subscribe("/control_logic/info",100000,Get_Motion_Info);
	ir_sub = n.subscribe("/core_sensors_ir/ir",100000,Get_Ir);
	camera_sub = n.subscribe("/vision/object",1,Place_Object);
	
	occ_pub = n.advertise<nav_msgs::OccupancyGrid>("/mapping/occ",1);
	map_pub = n.advertise<nav_msgs::MapMetaData>("/mapping/map",1);
	robo_pub = n.advertise<visualization_msgs::Marker>("/mapping/robot", 1);
	arrow_pub = n.advertise<visualization_msgs::Marker>("/mapping/robot_heading", 1);
	object_pub = n.advertise<visualization_msgs::Marker>("/mapping/object",1);
	corrected_map_pub = n.advertise<nav_msgs::OccupancyGrid>("/mapping/corrected_map",1);
	odo_pub = n.advertise<theia_services::corrected_odo>("/mapping/corrected_odo",1);
	talk_pub = n.advertise<std_msgs::String>("robot/talk",1);
	
	
	ros::ServiceServer map_sender = n.advertiseService("/mapping/ProcessedMap", provide_map);


	ROS_INFO("Started the mapping Node");

	ros::Rate loop_rate(freq);

	struct timeval start, end;
	
	for(int i=0; i<8; i++)
		ir[i]=0.0;
	
	while(ros::ok()){

		Occupancy_Grid=place_map(Occupancy_Grid,x_Current_Pose,y_Current_Pose,robot_delta_x,robot_delta_y,0);
		/*if(wall==-1){
			Correct_Map(robot_delta_x,robot_delta_y);
		}*/
		place_ir();
		update_robot();
		Send_Message();
		Send_Odometry();

		loop_rate.sleep();
		ros::spinOnce();


	}
	return 0;
}

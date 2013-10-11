#include "ros/ros.h"
#include <stdlib.h>
#include <cmath>
#include <control_hand/vw.h>
#include <control_hand/PidParams.h>
#include <robot_messages/coords.h>

ros::Publisher pub;
const float z_desired=60;
const float x_center=320;

/* Will hopefully perform better if just outputs pre-defined values */
const float VCONST=20;
const float WCONST=1; 

float p_x=0.02, p_z=2; // Controler parameters


float discretize(float num){ /* "ADC" */
	
	
	if(num<5 && num>-5)
		return 0;
	
	return num;
}

void change_params(const control_hand::PidParams::ConstPtr msg){
	
	p_x=msg->p_x;
	p_z=msg->p_z;
	
	ROS_INFO("Parameters changed as follows:\np_z= %.2f\np_x= %.2f\n",p_z,p_x);
	
}

void high_level_controller(const robot_messages::coords::ConstPtr msg){ /* Outputs a linear velocity command, v, proportional to the diference from z to z_desired and a linear velocity command, w, proportional to the difference from x to x_center */
	
	int error_x=0;
	float error_z=0.0;
	float x=msg->x;
	float z=msg->z;
	float v,w;
	control_hand::vw vw_msg;
	
	
	error_x=x_center-x;
	error_z=z-z_desired;
	
	if(error_z>100 || z<40)
		error_z=0; //ignore far away/too close objects

	v=p_z*error_z;
	w=p_x*error_x;
	
	if(v>VCONST){
		v=VCONST;
	}else{
		if(v<-VCONST){
			v=-VCONST;
		}else{
			v=0;
		}
	}
			
	if(w>WCONST){
		w=WCONST;
	}else{
		if(w<-WCONST){
			w=-WCONST;
		}else{
			w=0;
		}
	}
	
	
	vw_msg.v=v;
	vw_msg.w=w;
	
	pub.publish(vw_msg);
	
	
	
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "HandFollow");
	ros::NodeHandle n;
	ros::Subscriber coords;
	ros::Subscriber params;
	
	
	ros::Rate loop_rate(30);

	ROS_INFO("Started the HandFollow node");	
	
	pub=n.advertise<control_hand::vw>("/HandFollow/vw",1);
	coords=n.subscribe("/read_depth/xyz",1,high_level_controller);
	params=n.subscribe("/HandFollow/PidParams",1,change_params);
	
	while(ros::ok()){

		loop_rate.sleep();
		ros::spinOnce();

	}

	
	return 0;
}

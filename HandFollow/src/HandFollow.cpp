#include "ros/ros.h"
#include <stdlib.h>
#include <cmath>
#include <HandFollow/vw.h>
#include <HandFollow/PidParams.h>
#include <robot_messages/coords.h>

ros::Publisher pub;
const float z_desired=60;
const float x_center=240;
float p_x=0.25, p_z=0.5; // Controler parameters


float discretize(float num){ /* "ADC" */
	
	
	if(num<5 && num>-5)
		return 0;
	
	return num;
}

void change_params(const HandFollow::PidParams::ConstPtr msg){
	
	p_x=msg->p_x;
	p_z=msg->p_z;
	
	ROS_INFO("Parameters changed as follows\np_z= %.2f\np_x= %.2f\n",p_z,p_x);
	
}

void high_level_controller(const robot_messages::coords::ConstPtr msg){ /* Outputs a linear velocity command, v, proportional to the diference from z to z_desired and a linear velocity command, w, proportional to the difference from x to x_center */
	
	int error_x=0;
	float error_z=0.0;
	float x=msg->x;
	float z=msg->z;
	float v,w;
	HandFollow::vw vw_msg;
	
	
	error_x=x_center-x;
	error_z=z-z_desired;
	
	v=p_z*error_z;
	w=p_x*error_x;
	
	vw_msg.v=0//discretize(v);
	vw_msg.w=discretize(w);
	
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
	
	pub=n.advertise<HandFollow::vw>("/HandFollow/vw",1);
	coords=n.subscribe("/read_depth/xyz",1,high_level_controller);
	params=n.subscribe("/HandFollow/PidParams",1,change_params);
	
	while(ros::ok()){

		loop_rate.sleep();
		ros::spinOnce();

	}

	
	return 0;
}

#include "ros/ros.h"
#include <stdlib.h>
#include <cmath>
#include <HandFollow/vw.h>
#include <robot_messages/coords.h>

ros::Publisher pub;
const float z_desired=40;
const float x_center=240;

int sign(float num){
	
	if(num>0)
		return 1;
	if(num<0)
		return -1;
		
	return 0;
}

float discretize(float num){ /* outputs a 'smooth value' for the system */

	for(int i=1;i<10;i++){
		if(abs(num)<i*10)
			return (i-1)*10;
	}
	
	return 0;
}

void high_level_controller(const robot_messages::coords::ConstPtr msg){ /* Outputs a linear velocity command, v, proportional to the diference from z to z_desired and a linear velocity command, w, proportional to the difference from x to x_center */
	
	int error_x=0;
	float error_z=0.0;
	float p_x=1, p_z=1;
	float x=msg->x;
	float z=msg->z;
	float v,w;
	HandFollow::vw vw_msg;
	
	
	error_x=x_center-x;
	error_z=z-z_desired;
	
	v=p_z*error_z;
	w=p_x*error_x;
	
	vw_msg.v=discretize(v);
	vw_msg.w=discretize(w);
	
	pub.publish(vw_msg);
	
	
	
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "HandFollow");
	ros::NodeHandle n;
	ros::Subscriber coords;
	
	
	ros::Rate loop_rate(30);

	ROS_INFO("Started the HandFollow node");	
	
	pub=n.advertise<HandFollow::vw>("/HandFollow/vw",1);
	coords=n.subscribe("/read_depth/xyz",1,high_level_controller);
	
	while(ros::ok()){

		loop_rate.sleep();
		ros::spinOnce();

	}

	
	return 0;
}

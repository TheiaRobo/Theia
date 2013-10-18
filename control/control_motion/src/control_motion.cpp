#include "ros/ros.h"
#include <cmath>
#include <core_control_motor/vw.h>
#include <nav_msgs/Odometry.h>
#include <core_sensors/ir.h>

double freq=10.0;

int main(int argc, char ** argv){

	ros::init(argc, argv, "control_motion");
	ros::NodeHandle n;
	ros::Subscriber	odo_sub;
	ros::Subscriber ir_sub;
	ros::Publisher vw_pub;
	ros::Rate loop_rate(freq);
	
	vw_pub = n.advertise<core_control_motor::vw>("/control_motion/vw",1);
	
	
	while(ros::ok()){
		
		loop_rate.sleep();
		ros::spinOnce();
	}
	





	return 0;

}

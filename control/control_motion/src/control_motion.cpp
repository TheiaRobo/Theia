#include "ros/ros.h"
#include <cmath>
#include <core_control_motor/vw.h>
#include <nav_msgs/Odometry.h>
#include <core_sensors/ir.h>
#include <control_logic/MotionCommand.h>

double freq=10.0;


/* Function that should update the odometry values obtained from the core*/
void odo_proc(nav_msgs::Odometry::ConstPtr odo_msg){

}

/* Function that should update the IR values obtained from the core*/
void ir_proc(core_sensors::ir::ConstPtr ir_msg){

}

int main(int argc, char ** argv){

	ros::init(argc, argv, "control_motion");
	ros::NodeHandle n;
	ros::Subscriber	odo_sub;
	ros::Subscriber ir_sub;
	ros::Publisher vw_pub;
	ros::Rate loop_rate(freq);
	
        ros::ServiceClient brain = n.serviceClient<control_logic::MotionCommand>("control_logic/motion_command");
        control_logic::MotionCommand srv;
        srv.request.A = true;

	vw_pub = n.advertise<core_control_motor::vw>("/control_motion/vw",1);
	odo_sub = n.subscribe("/core_sensors/odometry",1,odo_proc);
	ir_sub = n.subscribe("/core_sensors/ir",1,ir_proc);
	
	
	
	while(ros::ok()){
		
          if(brain.call(srv)){
            ROS_INFO("service successful!!!");
          } else {
            ROS_INFO("service failed");
            return 1;
          }
 
		loop_rate.sleep();
		ros::spinOnce();
	}
	





	return 0;

}

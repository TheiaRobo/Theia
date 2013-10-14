#include <ros/ros.h> 
#include <std_msgs/Int32.h>
#include <core_control_motor/motorvel.h>
#include <core_control_odometry/odometry.h>
#include <visualization_msgs/Marker.h>
#include <cmath>
#include <string.h>
#include <signal.h>
#include <termios.h>


const float freq=100;
float x=0.0,y=0.0,theta=0.0;
const float L=23.0/2,R1=4.8,R2=4.8; // Values in cm

ros::Publisher odo_pub;
ros::Publisher marker_pub;

void update_odometry(const core_control_motor::motorvel::ConstPtr msg){
	
	float delta_s=0.0;
	float delta_theta=0.0;
	float baseline=2*L;
	core_control_odometry::odometry odo_msg;
	uint32_t shape;
	
	delta_s=(msg->vel1*R1+msg->vel2*R2)/(2*freq); //Travelled distance estimate for the vehicle
	delta_theta=(msg->vel2*R2-msg->vel1*R1)/(baseline*freq); // Estimated angle made by the vehicle
	
	
	// Following the odometry estimate from the book...
	x+=delta_s*cos(theta+delta_theta/2);
	y+=delta_s*sin(theta+delta_theta/2);
	theta+=delta_theta;
	
	odo_msg.header.stamp=ros::Time::now();
	odo_msg.x=x;
	odo_msg.y=y;
	odo_msg.theta=theta;
	
	shape=visualization_msgs::Marker::ARROW;
	
	odo_pub.publish(odo_msg);


}


int main(int argc, char ** argv){

	ros::init(argc,argv, "core_control_odometry"); 
	ros::NodeHandle n;	
	ros::Rate loop_rate(freq);
	ros::Subscriber encoder_sub;
	
	encoder_sub = n.subscribe("/core_control_filter/filtered_velocity",10,update_odometry);
	odo_pub = n.advertise<core_control_odometry::odometry>("/core_control_odometry/odometry",10);
	marker_pub = n.advertise<visualization_msgs::Marker>("/core_control_odometry/marker", 1);
	
	ROS_INFO("Started the core_control_odometry node");
	
	
	/* Main loop */
	while (ros::ok()){
	
	
	
		loop_rate.sleep();
		ros::spinOnce();
	}

	return 0;
}
	
		

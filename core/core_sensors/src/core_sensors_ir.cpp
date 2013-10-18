/*	core_sensors_ir node
*	
*	This node provides readings for every callibration IR in the robot
*
*
*/

#include <ros/ros.h>
#include <differential_drive/AnalogC.h>
#include <cmath> 
#include <cstring>
#include <stdlib.h>
#include <stdio.h>
#include <core_sensors/ir.h>


ros::Publisher dist_pub;
long num;


/*	convert
*
*	This function should be able to convert from raw sensor measurement to a
*	distance value in cm from the sensor to the obstacle. For this we need to
*	convert from the sensor curve to cm
*/
float convert(int raw){

	double p[6]={-1.4*pow(10,-11),2.47*pow(10,-8),-1.7*pow(10,-5),5.78*pow(10,-3),-1.01*pow(10,0),85.2};

	return (double) p[0]*pow(raw,5)+p[1]*pow(raw,4)+p[2]*pow(raw,3)+p[3]*pow(raw,2)+p[4]*pow(raw,1)+p[5]*pow(raw,0);

}

void get_distance(const differential_drive::AnalogC::ConstPtr msg){
	core_sensors::ir dist_msg;
	
	dist_msg.dist[0]=convert(msg->ch1);
	dist_msg.dist[1]=convert(msg->ch2);
	dist_msg.dist[2]=convert(msg->ch3);
	dist_msg.dist[3]=convert(msg->ch4);
	dist_msg.dist[4]=convert(msg->ch5);
	dist_msg.dist[5]=convert(msg->ch6);
	dist_msg.dist[6]=convert(msg->ch7);
	dist_msg.dist[7]=convert(msg->ch8);

	
	dist_pub.publish(dist_msg);

}

int main(int argc, char ** argv){
	
	ros::init(argc,argv,"core_sensors_ir");
	ros::NodeHandle n;			
	ros::Subscriber analog_sub; 		
	ros::Rate loop_rate(100);
	
	
	ROS_INFO("Started the core_sensors_ir Node");
	
	analog_sub = n.subscribe("/sensors/ADC",1,get_distance);
	dist_pub = n.advertise<core_sensors::ir>("/core_sensors_ir/ir",1);

	
	/* Main loop */
	while (ros::ok()){
	
		
	
		loop_rate.sleep();
		ros::spinOnce();
	}

	return 0;
}
	
		

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
double convert(int raw,int sensor){

	double p[8][6];

	for(int i=0; i<8;i++)
		for(int j=0; j<6;j++)
			p[i][j]=0;

	// poly values for frontal top left sensor
	p[0][0]=-6.16*pow(10,-11);
	p[0][1]=8.91*pow(10,-8);
	p[0][2]=-5.06*pow(10,-5);
	p[0][3]=1.42*pow(10,-2);
	p[0][4]=-2.03*pow(10,0);
	p[0][5]=1.27*pow(10,2);

	// poly values for frontal top right sensor
	p[1][0]=-3.02*pow(10,-11);
	p[1][1]=4.54*pow(10,-8);
	p[1][2]=-2.69*pow(10,-5);
	p[1][3]=7.98*pow(10,-3);
	p[1][4]=-1.24*pow(10,0);
	p[1][5]=8.85*pow(10,1);

	return (double) p[sensor-1][0]*pow(raw,5)+p[sensor-1][1]*pow(raw,4)+p[sensor-1][2]*pow(raw,3)+p[sensor-1][3]*pow(raw,2)+p[sensor-1][4]*pow(raw,1)+p[sensor-1][5]*pow(raw,0);

}

void get_distance(const differential_drive::AnalogC::ConstPtr msg){
	core_sensors::ir dist_msg;
	
	dist_msg.dist[0]=convert(msg->ch1,1);
	dist_msg.dist[1]=convert(msg->ch2,2);
	dist_msg.dist[2]=convert(msg->ch3,3);
	dist_msg.dist[3]=convert(msg->ch4,4);
	dist_msg.dist[4]=convert(msg->ch5,5);
	dist_msg.dist[5]=convert(msg->ch6,6);
	dist_msg.dist[6]=convert(msg->ch7,7);
	dist_msg.dist[7]=convert(msg->ch8,8);

	
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
	
		

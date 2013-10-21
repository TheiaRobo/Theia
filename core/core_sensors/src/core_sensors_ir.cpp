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

const int AVG_MAX=9;
double avg[8][AVG_MAX];



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
	p[0][0]=-3.30*pow(10,-11);
	p[0][1]=4.96*pow(10,-8);
	p[0][2]=-2.95*pow(10,-5);
	p[0][3]=8.81*pow(10,-3);
	p[0][4]=-1.3725*pow(10,0);
	p[0][5]=9.71*pow(10,1);

	// poly values for frontal top right sensor
	
	
	p[1][0]=-3.30*pow(10,-11);
	p[1][1]=4.96*pow(10,-8);
	p[1][2]=-2.95*pow(10,-5);
	p[1][3]=8.81*pow(10,-3);
	p[1][4]=-1.3725*pow(10,0);
	p[1][5]=9.71*pow(10,1);
	
	/*p[1][0]=-1.90*pow(10,-11);
	p[1][1]=2.96*pow(10,-8);
	p[1][2]=-1.83*pow(10,-5);
	p[1][3]=5.75*pow(10,-3);
	p[1][4]=-9.64*pow(10,-1);
	p[1][5]=7.61*pow(10,1);*/
	
	// side facing left front
	
	p[2][0]=-3.30*pow(10,-11);
	p[2][1]=4.96*pow(10,-8);
	p[2][2]=-2.95*pow(10,-5);
	p[2][3]=8.81*pow(10,-3);
	p[2][4]=-1.3725*pow(10,0);
	p[2][5]=9.71*pow(10,1);
	
	
	/*p[2][0]=-5.12*pow(10,-11);
	p[2][1]=7.33*pow(10,-8);
	p[2][2]=-4.15*pow(10,-5);
	p[2][3]=1.16*pow(10,-2);
	p[2][4]=-1.73*pow(10,0);
	p[2][5]=1.15*pow(10,2);*/
	
	// side facing left rear
	
	p[3][0]=-3.30*pow(10,-11);
	p[3][1]=4.96*pow(10,-8);
	p[3][2]=-2.95*pow(10,-5);
	p[3][3]=8.81*pow(10,-3);
	p[3][4]=-1.3725*pow(10,0);
	p[3][5]=9.71*pow(10,1);
	
	/*p[3][0]=-1.21*pow(10,-10);
	p[3][1]=1.56*pow(10,-7);
	p[3][2]=-7.95*pow(10,-5);
	p[3][3]=1.99*pow(10,-2);
	p[3][4]=-2.53*pow(10,0);
	p[3][5]=1.40*pow(10,2);*/
	
	// side facing right front
	
	p[4][0]=-3.30*pow(10,-11);
	p[4][1]=4.96*pow(10,-8);
	p[4][2]=-2.95*pow(10,-5);
	p[4][3]=8.81*pow(10,-3);
	p[4][4]=-1.3725*pow(10,0);
	p[4][5]=9.71*pow(10,1);
	
	/*p[4][0]=-5.52*pow(10,-11);
	p[4][1]=7.15*pow(10,-8);
	p[4][2]=-3.67*pow(10,-5);
	p[4][3]=9.54*pow(10,-3);
	p[4][4]=-1.32*pow(10,0);
	p[4][5]=8.74*pow(10,1);*/
	
	// side facing right rear
	
	p[5][0]=-3.30*pow(10,-11);
	p[5][1]=4.96*pow(10,-8);
	p[5][2]=-2.95*pow(10,-5);
	p[5][3]=8.81*pow(10,-3);
	p[5][4]=-1.3725*pow(10,0);
	p[5][5]=9.71*pow(10,1);
	
	/*p[5][0]=-8.73*pow(10,-11);
	p[5][1]=1.12*pow(10,-7);
	p[5][2]=-5.69*pow(10,-5);
	p[5][3]=1.44*pow(10,-2);
	p[5][4]=-1.88*pow(10,0);
	p[5][5]=1.11*pow(10,2);*/
	
	return (double) p[sensor-1][0]*pow(raw,5)+p[sensor-1][1]*pow(raw,4)+p[sensor-1][2]*pow(raw,3)+p[sensor-1][3]*pow(raw,2)+p[sensor-1][4]*pow(raw,1)+p[sensor-1][5]*pow(raw,0);

}

int compare (const void * a, const void * b)
{
  return ( *(int*)a - *(int*)b );
}

void get_distance(const differential_drive::AnalogC::ConstPtr msg){
	core_sensors::ir dist_msg;
	
	
	for(int i=0; i<7; i++)
		for(int j=0; j<AVG_MAX-1; j++)
			avg[i][j+1]=avg[i][j];
	
	
	avg[0][0]=convert(msg->ch1,1);
	avg[1][0]=convert(msg->ch2,2);
	avg[2][0]=convert(msg->ch3,3);
	avg[3][0]=convert(msg->ch4,4);
	avg[4][0]=convert(msg->ch5,5);
	avg[5][0]=convert(msg->ch6,6);
	avg[6][0]=convert(msg->ch7,7);
	avg[7][0]=convert(msg->ch8,8);
	
	
	for(int i=0; i<8;i++)
		qsort((void*) &avg[i],AVG_MAX,sizeof(double),compare);
	
	for(int i=0; i<8;i++){
		dist_msg.dist[i]=avg[i][AVG_MAX/2];
	}
		
	
	/*dist_msg.dist[0]=convert(msg->ch1,1);
	dist_msg.dist[1]=convert(msg->ch2,2);
	dist_msg.dist[2]=convert(msg->ch3,3);
	dist_msg.dist[3]=convert(msg->ch4,4);
	dist_msg.dist[4]=convert(msg->ch5,5);
	dist_msg.dist[5]=convert(msg->ch6,6);
	dist_msg.dist[6]=convert(msg->ch7,7);
	dist_msg.dist[7]=convert(msg->ch8,8);*/

	
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
	
	for(int i=0; i<8; i++)
		for(int j=0; j<3; j++)
			avg[i][j]=0;
	
	/* Main loop */
	while (ros::ok()){
	
		
	
		loop_rate.sleep();
		ros::spinOnce();
	}

	return 0;
}
	
		

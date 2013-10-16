/*	core_sensors_ir node
*	
*	This node should be called as follows: $rosrun core_sensors core_sensors_ir ir_num
*	This allow us to use the callibration values for each ir (they differ a bit on their curves)
*
*/


#include <ros/ros.h>
#include <differential_drive/AnalogC.h>
#include <cmath> 
#include <cstring>
#include <stdlib.h>
#include <stdio.h>

void get_distance(const differential_drive::AnalogC::ConstPtr msg){

}

int main(int argc, char ** argv){

	long num;
	char node_name[25]="core_sensors_ir_",node_num[20];
	
	
	if(argc < 2){
		ROS_ERROR("Usage: rosrun core_sensors core_sensors_ir ir_num");
		exit(0);
	}
	
	
	if((num=strtol(argv[1],NULL,0))==0){
		ROS_ERROR("ir_num must be an integer");
		exit(0);
	}
	
	sprintf(node_num,"%li",num);
	strcat(node_name,node_num);
	
	ros::init(argc,argv,node_name);
	ros::NodeHandle n;			
	ros::Subscriber analog_sub; 		
	ros::Rate loop_rate(100);
	
	
	ROS_INFO("Started the %s Node",node_name);
	
	analog_sub = n.subscribe("/sensors/ADC",1,get_distance);

	
	/* Main loop */
	while (ros::ok()){
	
		
	
		loop_rate.sleep();
		ros::spinOnce();
	}

	return 0;
}
	
		

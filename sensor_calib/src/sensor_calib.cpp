#include <ros/ros.h>
#include <differential_drive/AnalogC.h>
#include <stdio.h>
#include <iostream>
#include <fstream>

using namespace std;
int counter=1;

void print_sensor(const differential_drive::AnalogC::ConstPtr msg){

	// 5 values per line, regarding the same sensor distance
	
	ofstream myfile;

	ROS_INFO("Sensor values:");
	ROS_INFO("Ch1: %d",msg->ch1);
	ROS_INFO("Ch2: %d",msg->ch2);
	ROS_INFO("Ch3: %d",msg->ch3);
	ROS_INFO("Ch4: %d",msg->ch4);
	ROS_INFO("Ch5: %d",msg->ch5);
	ROS_INFO("Ch6: %d",msg->ch6);
	ROS_INFO("Press any key to continue...");
	
	getchar();

	myfile.open("data_1.txt",ios::app);
	myfile << msg->ch1;
	
	if(counter==5){
		myfile << "\n";
	}
		
	myfile.close();

	myfile.open("data_2.txt",ios::app);
	myfile << msg->ch2;
	
	if(counter==5){
		myfile << "\n";
		counter=0;
	}
	counter++;
		
	myfile.close();
	

}



int main(int argc, char ** argv){
	
	ros::init(argc,argv,"sensor_calib");
	ros::NodeHandle n;
	ros::Rate loop_rate(100);
	
	ros::Subscriber sub;
	
	
	sub=n.subscribe("/sensors/ADC",1,print_sensor);
	
	while(ros::ok()){
	
		
		
		
		ros::spinOnce();
		loop_rate.sleep();
	
	}


}

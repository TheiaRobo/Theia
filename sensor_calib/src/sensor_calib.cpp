#include <ros/ros.h>
#include <differential_drive/AnalogC.h>
#include <stdio.h>
#include <iostream>
#include <fstream>

using namespace std;

int mes=1;
int counter=1;

void print_sensor(const differential_drive::AnalogC::ConstPtr msg){

	
	ros::Duration refresh(0.1);
	// 10 values per line, regarding the same sensor distance
	
	ofstream myfile;

	if(counter==1){
		ROS_INFO("Measurement number %d\nPress any key to continue...",mes);
	
		getchar();
	}
	
	if(counter!=1){//discard first reading
		myfile.open("data_5.txt",ios::app);
		myfile << msg->ch5;
		myfile << "\t";

		if(counter==10){
			myfile << "\n";
		}
	
		myfile.close();

		myfile.open("data_6.txt",ios::app);
		myfile << msg->ch6;
		myfile << "\t";

		if(counter==10){
			myfile << "\n";
			counter = 0;
		}
		myfile.close();
	}
	refresh.sleep();
	counter++;
	
	
	if(counter==1)
		mes++;

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

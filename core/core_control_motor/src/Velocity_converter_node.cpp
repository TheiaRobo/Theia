#include "ros/ros.h"
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <differential_drive/PWM.h>
#include <differential_drive/Encoders.h>
#include <string.h>
#include <core_control_motor/motorvel.h>
#define PI 3.1415926f

using namespace differential_drive;
ros::Subscriber sub2;
ros::Publisher  filter_enc_pub;
	
//Base updaterate of encoder topic is one msg per 100 ms.
long encoder_interval = 100;

core_control_motor::motorvel convert_msg;

void convert_encoders_value(const Encoders::ConstPtr &msg)
{
	double ticks_to_centimeter=PI/3.6;
	
		
	convert_msg.delta_encoder1 = msg->delta_encoder1*ticks_to_centimeter;
	convert_msg.delta_encoder2 = msg->delta_encoder2*ticks_to_centimeter;


}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "core_control_converter");
	ros::NodeHandle n;
	
	sub2 = n.subscribe("/motion/Encoders", 100, convert_encoders_value);
	convert_enc_pub = n.advertise<core_control_motor::motorvel>("/core_control_motor/velocity",100);
	
	ros::spinOnce();
	ros::Rate loop_rate(100);
	ros::Time t_start = ros::Time::now();
	
	

	while(ros::ok()){

		convert_msg.timestamp = int((ros::Time::now()-t_start).toSec()*1000.0);

		convert_enc_pub.publish(filtered_msg);

		ros::spinOnce();
		loop_rate.sleep();

	}
	return 0;
}

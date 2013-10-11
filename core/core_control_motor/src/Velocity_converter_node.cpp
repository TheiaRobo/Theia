#include "ros/ros.h"
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <differential_drive/PWM.h>
#include <differential_drive/Encoders.h>
#include <string.h>
#include <core_control_motor/motorvel.h>

using namespace differential_drive;
ros::Subscriber sub2;
ros::Publisher  filter_enc_pub;

const float PI=3.1415926f;
const float freq=100.0;

core_control_motor::motorvel convert_msg;

void convert_encoders_value(const Encoders::ConstPtr &msg)
{
	double ticks_to_speed=-2*PI/360;
	
		
	convert_msg.vel1 = msg->delta_encoder1*ticks_to_speed*freq;
	convert_msg.vel2 = msg->delta_encoder2*ticks_to_speed*freq;


}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "core_control_converter");
	ros::NodeHandle n;
	ros::Publisher convert_enc_pub;
	
	sub2 = n.subscribe("/motion/Encoders", 100, convert_encoders_value);
	convert_enc_pub = n.advertise<core_control_motor::motorvel>("/core_control_motor/velocity",100);
	
	ros::spinOnce();
	ros::Rate loop_rate(freq);
	ros::Time t_start = ros::Time::now();
	
	

	while(ros::ok()){

		convert_msg.timestamp = int((ros::Time::now()-t_start).toSec()*1000.0);

		convert_enc_pub.publish(convert_msg);

		ros::spinOnce();
		loop_rate.sleep();

	}
	return 0;
}

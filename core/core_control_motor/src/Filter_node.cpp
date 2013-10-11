#include "ros/ros.h"
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <differential_drive/PWM.h>
#include <differential_drive/Encoders.h>
#include <core_control_motor/motorvel.h>
#include <string.h>

using namespace differential_drive;

ros::Subscriber sub2;
ros::Publisher  filter_enc_pub;

core_control_motor::motorvel filtered_msg;

void filter_encoders_value(const core_control_motor::motorvel::ConstPtr &msg)
{

	filtered_msg.vel1 = msg->vel1;
	filtered_msg.vel2 = msg->vel2;
	
}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "core_control_filter");
	ros::NodeHandle n;
	
	sub2 = n.subscribe("/core_control_motor/velocity", 100, filter_encoders_value);
	filter_enc_pub = n.advertise<core_control_motor::motorvel>("/core_control_filter/filtered_velocity",100000);
	ros::spinOnce();
	ros::Rate loop_rate(100);
	ros::Time t_start = ros::Time::now();

	while(ros::ok()){

		filtered_msg.timestamp = int((ros::Time::now()-t_start).toSec()*1000.0);

		filter_enc_pub.publish(filtered_msg);

		ros::spinOnce();
		loop_rate.sleep();

	}
	return 0;
}

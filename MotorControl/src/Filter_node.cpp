#include "ros/ros.h"
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <differential_drive/PWM.h>
#include <differential_drive/Encoders.h>
#include <string.h>

using namespace differential_drive;
ros::Subscriber sub2;
ros::Publisher  filter_enc_pub;
	
//Base updaterate of encoder topic is one msg per 100 ms.
long encoder_interval = 100;

Encoders filtered_msg;

void filter_encoders_value(const Encoders::ConstPtr &msg)
{
	/*	static float encoder_value_old;
	static float encoder_value_old;
*/
	filtered_msg.delta_encoder1 = msg->delta_encoder1;
	filtered_msg.delta_encoder2 = msg->delta_encoder2;
	filtered_msg.timestamp = msg->timestamp;
	printf("%d:got encoder L:%d , false R:%d\n",filtered_msg.timestamp,filtered_msg.delta_encoder1,filtered_msg.delta_encoder2);
}

void updateEncoders(float time)
{
	time=time+1;
	time=time-1;
}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "Filter_node");
	ros::NodeHandle n;
	sub2 = n.subscribe("/motion/Encoders", 100, filter_encoders_value);
	filter_enc_pub = n.advertise<Encoders>("/motors/filtered_encoders"	, 100000);
	ros::spinOnce();
	ros::Rate loop_rate(100);
	ros::Time t_start = ros::Time::now();

	while(ros::ok()){
		//filtered_msg.delta_encoder1 = 1;
		//filtered_msg.delta_encoder2 = 1;
		filtered_msg.timestamp = int((ros::Time::now()-t_start).toSec()*1000.0);
/*		printf("Timestamp:%d\n", filtered_msg.timestamp);
		printf("right:%d\n", int(filtered_msg.delta_encoder1));
		printf("left:%d\n", int(filtered_msg.delta_encoder2));
*/
		filter_enc_pub.publish(filtered_msg);

		ros::spinOnce();
		loop_rate.sleep();
		updateEncoders(0.01);
	}
	return 0;
}

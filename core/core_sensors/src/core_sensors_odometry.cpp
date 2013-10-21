#include <ros/ros.h> 
#include <std_msgs/Int32.h>
#include <core_control_motor/motorvel.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <cmath>
#include <string.h>
#include <signal.h>
#include <termios.h>


const double freq=100;
double x=0.0,y=0.0,theta=0.0;
double L=21.35/2,R1=5.0,R2=5.0; // Values in cm

ros::Publisher odo_pub;


void update_odometry(const core_control_motor::motorvel::ConstPtr msg){
	
	// Get params from server
	ros::param::getCached("core/L",L);
	ros::param::getCached("core/R1",R1);
	ros::param::getCached("core/R2",R2);	
	
	
	double delta_s=0.0;
	double delta_theta=0.0;
	double baseline=2*L;
	double dist1=0.0,dist2=0.0;
	geometry_msgs::Quaternion odom_quat;
	geometry_msgs::TransformStamped odom_trans;
	tf::TransformBroadcaster odom_broadcaster;
	nav_msgs::Odometry odom;
	
	dist1=msg->vel1*R1/freq; //Distance travelled by left wheel
	dist2=msg->vel2*R2/freq;
	
	delta_s=(dist1+dist2)/2; //Travelled distance estimate for the vehicle
	delta_theta=(dist2-dist1)/baseline; // Estimated angle made by the vehicle
	
	theta+=delta_theta;
	x+=delta_s*cos(theta);
	y+=delta_s*sin(theta);
	
	ROS_INFO("theta: %.4f",theta);
	
	//since all odometry is 6DOF we'll need a quaternion created from yaw
	odom_quat = tf::createQuaternionMsgFromYaw(theta);
	
	//first, we'll publish the transform over tf
	odom_trans.header.stamp = ros::Time::now();
	odom_trans.header.frame_id = "odometry";
	odom_trans.child_frame_id = "base_link";
	
	odom_trans.transform.translation.x = x;
	odom_trans.transform.translation.y = y;
	odom_trans.transform.translation.z = 0.0;
	odom_trans.transform.rotation = odom_quat;
	
	//send the transform
	odom_broadcaster.sendTransform(odom_trans);
	
	//next, we'll publish the odometry message over ROS
	odom.header.stamp=ros::Time::now();
	odom.header.frame_id= "odometry";
	
	//set the position
	odom.pose.pose.position.x=x/100; // so that one unit here is treated as cm
	odom.pose.pose.position.y=y/100;
	odom.pose.pose.position.z=0.0;
	odom.pose.pose.orientation=odom_quat;
	
	//set velocity
	
	odom.child_frame_id = "base_link";
	// here we can introduce the velocity estimates
	odom.twist.twist.linear.x = 0; 
	odom.twist.twist.linear.y = 0;
	odom.twist.twist.angular.z = delta_theta;
	
	
	odo_pub.publish(odom);


}


int main(int argc, char ** argv){

	ros::init(argc,argv, "core_sensors_odometry"); 
	ros::NodeHandle n;	
	ros::Rate loop_rate(freq);
	ros::Subscriber encoder_sub;
	
	encoder_sub = n.subscribe("/core_control_filter/filtered_velocity",10,update_odometry);
	odo_pub = n.advertise<nav_msgs::Odometry>("/core_sensors_odometry/odometry", 10);
	
	ROS_INFO("Started the core_sensors_odometry node");
	
	
	/* Main loop */
	while (ros::ok()){
	
	
	
		loop_rate.sleep();
		ros::spinOnce();
	}

	return 0;
}
	
		

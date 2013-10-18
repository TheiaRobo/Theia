#include "ros/ros.h"
#include <stdlib.h>
#include <cmath>
#include <core_control_motor/vw.h>
#include <teleop_msgs/State.h>

double v=0.0;
double w=0.0;
double prev_key[2]={0,0};
int is_teleop=0;

double V_MAX=10.0;
double W_MAX=1;
double freq=10.0;


void HandFollow_up(const core_control_motor::vw::ConstPtr msg){
	
	if(!is_teleop){	// Teleoperation overrides the controller
		v=msg->v;
		w=msg->w;
	}
	
}

/* Teleoperation Update: will generate a linear or angular velocity, based on keyboard commands. Front: constant v>0 value. Back: constant v<0 value. Left: constant w>0 value. Right: constant w>0 value*/
void Teleop_up(const teleop_msgs::State::ConstPtr msg){

	double value[2]={0,0};
	
	value[0]=msg->axes[0].value;
	value[1]=msg->axes[1].value;
	
	ros::param::getCached("/control_mux/V_MAX",V_MAX);
	ros::param::getCached("/control_mux/W_MAX",W_MAX);
	
	if(value[0]>prev_key[0]){ // new key press
		v+=V_MAX;
		w=0;

	}else{
		if(value[0]<prev_key[0]){
			v-=V_MAX;
			w=0;
		}else{
	
			if(value[1]>prev_key[1]){
				w+=W_MAX;
				v=0;
			}else{
				if(value[1]<prev_key[1]){
					w-=W_MAX;
					v=0;	
				}
			}
		}
	}
	
	prev_key[0]=value[0];
	prev_key[1]=value[1];
	
	if(v>V_MAX)
		v=V_MAX;
	else
		if(v<-V_MAX)
			v=-V_MAX;
			
	if(w>W_MAX)
		w=W_MAX;
	else
		if(w<-W_MAX)
			w=-W_MAX;
			
		

}
	
	

int main(int argc, char **argv)
{
	ros::init(argc, argv, "control_mux");
	ros::NodeHandle n;
	ros::Subscriber Hand_sub;
	ros::Subscriber Teleop_sub;
	ros::Publisher pub;
	core_control_motor::vw msg;
	
	ros::Rate loop_rate(freq);

	ROS_INFO("Started the control_mux node");	
	
	pub=n.advertise<core_control_motor::vw>("/control_mux/vw",1);
	Hand_sub=n.subscribe("/control_motion/vw",1,HandFollow_up);
	Teleop_sub=n.subscribe("/teleop_source_node/teleop",1,Teleop_up);
	
	
	while(ros::ok()){
		
		if(Teleop_sub.getNumPublishers()>0) /* If teleop data is being published, enter teleoperation mode */
			is_teleop=1;
		else
			is_teleop=0;
			
		msg.v=v;
		msg.w=w;
		pub.publish(msg);
		
		loop_rate.sleep();
		ros::spinOnce();

	}

	
	return 0;
}

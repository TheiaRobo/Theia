#include "ros/ros.h"
#include <stdlib.h>
#include <cmath>
#include <control_hand/vw.h>
#include <robot_messages/coords.h>
#include <teleop_msgs/State.h>

float v=0.0;
float w=0.0;
float prev_key[2]={0,0};
int is_teleop=0;

const float V_MAX=40.0;
const float W_MAX=1;


void HandFollow_up(const control_hand::vw::ConstPtr msg){
	
	if(!is_teleop){	// Teleoperation overrides the controller
		v=msg->v;
		w=msg->w;
	}
	
}

/* Teleoperation Update: will generate a linear or angular velocity, based on keyboard commands. Front: constant v>0 value. Back: constant v<0 value. Left: constant w>0 value. Right: constant w>0 value*/
void Teleop_up(const teleop_msgs::State::ConstPtr msg){

	float value[2]={0,0};
	
	value[0]=msg->axes[0].value;
	value[1]=msg->axes[1].value;
	
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
	
	if(v>2*V_MAX)
		v=2*V_MAX;
	else
		if(v<-2*V_MAX)
			v=-2*V_MAX;
			
	if(w>2*W_MAX)
		w=2*W_MAX;
	else
		if(w<-2*W_MAX)
			w=-2*W_MAX;
			
		

}
	
	

int main(int argc, char **argv)
{
	ros::init(argc, argv, "ControlMux");
	ros::NodeHandle n;
	ros::Subscriber Hand_sub;
	ros::Subscriber Teleop_sub;
	ros::Publisher pub;
	control_hand::vw msg;
	
	ros::Rate loop_rate(30);

	ROS_INFO("Started the ControlMux node");	
	
	pub=n.advertise<control_hand::vw>("/ControlMux/vw",1);
	Hand_sub=n.subscribe("/HandFollow/vw",1,HandFollow_up);
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

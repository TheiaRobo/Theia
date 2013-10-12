#include <ros/ros.h> // needs rosccp dep in the Manifest
#include <std_msgs/Int32.h>
#include <core_control_motor/vw.h> 
#include <differential_drive/PWM.h> // needs differential_drive in the Manifest
#include <differential_drive/Encoders.h>
#include <cmath> // Has some cpp math functions than can be used for the PID controller
#include <core_control_motor/motorvel.h>
#include <core_control_motor/pid.h>
#include <string.h>
//#include <geometry_msgs/Twist.h>
#include <signal.h>
#include <termios.h>

using namespace differential_drive;
float Ref1=0, Ref2=0;

const float L=23/2,R1=4.8,R2=4.8; 	// length between the wheels(L) and diameter of the the wheels (R), values in cm!

/* PID controller values */

float P1=7.0,P2=7.0,I1=100.0,I2=100.0,D1=20.0,D2=20.0; // This values should be allowed to change after receiving a debug msg

float v_left=0;
float v_right=0;
int timestamp;


ros::Publisher pwm_pub;

	
/* this message will store the PWM values to be sent to the Arduino. It should be updated on the PID function and sent on the main loop*/
differential_drive::PWM pwm_msg;


void receive_encoder(const core_control_motor::motorvel::ConstPtr msg)
{
	
	v_left = msg->vel1;
	v_right = msg->vel2;



}

void update_params(const core_control_motor::pid::ConstPtr msg){
	
	P1=msg->p1;
	P2=msg->p2;
	I1=msg->i1;
	I2=msg->i2;
	D1=msg->d1;
	D2=msg->d2;
	
	ROS_INFO("New PID params:\nP1: %.2f; P2: %.2f\nI1: %.2f; I2: %.2f\nD1: %.2f; D2: %.2f\n",P1,P2,I1,I2,D1,D2);
	
}

/* This function is called when the node receives a message from the /ControlMux/vw topic and converts the v and w values into the references for each motor. These references should be in m/s or cm/s or equivalent */
void RefConverter(const core_control_motor::vw::ConstPtr &msg){ 	

	float angv, linv;
	
	linv=msg->v;
	angv=msg->w;
	
	/* Reference values obtained from the equations for the linear and angular velocity. They are the spinning speed of the wheel */
	Ref1=(linv-angv*L)/R1;	
	Ref2=(linv+angv*L)/R2;
	
	// Debug message. Useful when performing PID tunning
	//ROS_INFO("REF1: %.4f\nREF2: %.4f\n",Ref1,Ref2);
		
}

/* The PID algorithm goes here */	
int PID_control(float P,float I,float D,float * integrator_sum, float * differentiator_val,float Ref,float vel){
	
	float error=Ref-vel;  		
	float P_part=0,I_part=0,D_part=0, total=0;
	
	
	//integrator_sum  +=(error*dt);	// I Part
	(*integrator_sum)+=error*0.01;
	
	//differentiator_val =(error-previous error)/dt; // D Part
	(*differentiator_val)=(error-(*differentiator_val))*100;
	
	P_part=P*error;
	I_part=I*(*integrator_sum);
	
	/*if(Ref==0){
		I_part=0; /* Will assume that the wheels will not turn when the input value is 0 *//*
		*integrator_sum=0;
	 }*/
	
	total=P_part+I_part+D_part;
	
	if(total>255) // If this occurs, either the reference is irrealistic, or the I part is messing up
		total=255;
	if(total<-255)
		total=-255;
	
	return round(total);//+ (I*Integrator)+(D*Derivative);
}
	


int main(int argc, char ** argv){

	ros::init(argc,argv, "core_control_motor"); //initialise ros and the arguments
	ros::NodeHandle n;			// Node handler 
	ros::Subscriber vw_sub; 		
	ros::Subscriber	enc_sub;
	ros::Subscriber param_sub;	
	

	float cum1=0.0,cum2=0.0,diff1=0.0,diff2=0.0;

	
	/* Loop rate of 100Hz to comply with the encoders update frequency */
	ros::Rate loop_rate(100);
	ROS_INFO("Started the core_control_motor Node");
	
	vw_sub = n.subscribe("/control_mux/vw",1,RefConverter);
	pwm_pub = n.advertise<differential_drive::PWM>("/motion/PWM",1);
	enc_sub = n.subscribe("/core_control_filter/filtered_velocity", 1000, receive_encoder);
	param_sub=n.subscribe("/core_control_motor/pid",1,update_params);
	
	/* Main loop */
	while (ros::ok()){
	
		pwm_msg.PWM1=PID_control(P1,I1,D1,&cum1,&diff1,Ref1,v_left);
		pwm_msg.PWM2=PID_control(P2,I2,D2,&cum2,&diff2,Ref2,v_right);
		pwm_msg.header.stamp=ros::Time::now();
		pwm_pub.publish(pwm_msg);
	
		loop_rate.sleep();
		ros::spinOnce();
	}

	return 0;
}
	
		

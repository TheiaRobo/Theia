#include <ros/ros.h> // needs rosccp dep in the Manifest
#include <std_msgs/Int32.h>
#include <HandFollow/vw.h> // needs HandFollow dep in the Manifest. Allows to use the vw msg defined in the HandFollow package
#include <differential_drive/PWM.h> // needs differential_drive in the Manifest
#include <differential_drive/Encoders.h>
#include <cmath> // Has some cpp math functions than can be used for the PID controller

#include <string.h>
//#include <geometry_msgs/Twist.h>
#include <signal.h>
#include <termios.h>

using namespace differential_drive;
float Ref1=0, Ref2=0;

const float L=23,R1=4.8,R2=4.8; 	// length between the wheels(L) and diameter of the the wheels (R), values in cm!

/* PID controller values */

float P1=1.0,P2=1.0,I1=0.0,I2=0.0,D1=0.0,D2=0.0; // This values should be allowed to change after receiving a debug msg
float cum1=0.0,cum2=0.0,diff1=0.0,diff2=0.0;

int left;
int right;
int timestamp;

// Functions initialization
//void receive_encoder(const Encoders::ConstPtr &msg1);

ros::Subscriber vw_sub; 			
ros::Publisher pwm_pub;
ros::Subscriber	enc_sub;
	
/* this message will store the PWM values to be sent to the Arduino. It should be updated on the PID function and sent on the main loop*/
differential_drive::PWM pwm_msg;
//differential_drive::Encoders msg1; 


void receive_encoder(const Encoders::ConstPtr &msg1)
{
	//static ros::Time t_start1 = ros::Time::now();
	right = msg1->delta_encoder1;
	left = msg1->delta_encoder2;
	timestamp = msg1->timestamp;
	printf("%d:got encoder L:%d , false R:%d\n",timestamp,left,right);

}

/* This function is called when the node receives a message from the /ControlMux/vw topic and converts the v and w values into the references for each motor. These references should be in m/s or cm/s or equivalent */
void RefConverter(const HandFollow::vw::ConstPtr &msg){ 	

	float angv, linv;
	
	linv=msg->v;
	angv=msg->w;
	
	/* Reference values obtained from the equations for the linear and angular velocity */
	Ref1=(linv-angv*L)/R1; 	
	Ref2=(linv+angv*L)/R2;
	
	/* This code should be temporary, since this function is supposed to leave the References as a m/s (or equivalent) value. Conversion to PWM for the output should be done only after all the calculations of the PID algorithm */	
	if (Ref1 >255)
		Ref1=255;
		
	if (Ref1 <-255)
		Ref1=-255;
		
	if (Ref2 >255)
		Ref2=255;
		
	if (Ref2 <-255)
		Ref2=-255;
		
}

/* The PID algorithm goes here */	
int PID_control(float P,float I,float D,float integrator_sum, float differentiator_val,float Ref,float vel){
	float error=Ref-vel;  		// P Part
	//integrator_sum  +=(error*dt);	// I Part
	//differentiator_val =(error-previous error)/dt; // D Part
	return round((P*error));//+ (I*Integrator)+(D*Derivative);
}
	

int main(int argc, char ** argv){
	ros::init(argc,argv, "MotorControl_node"); //initialise ros and the arguments
	ros::NodeHandle n;			// Node handler 

	
	/* Loop rate of 100Hz to comply with the encoders update frequency */
	ros::Rate loop_rate(100);
	ROS_INFO("Started the Motor Control Node");
	
	vw_sub = n.subscribe("/ControlMux/vw",1,RefConverter);
	pwm_pub = n.advertise<differential_drive::PWM>("/motion/PWM",1);
	enc_sub = n.subscribe("/motors/filtered_encoders", 1000, receive_encoder);
	
	/* Main loop */
	while (ros::ok()){
	
		pwm_msg.PWM1=PID_control(P1,I1,D1,cum1,diff1,Ref1,0);
		pwm_msg.PWM2=PID_control(P2,I2,D2,cum2,diff2,Ref2,0);
		pwm_msg.header.stamp=ros::Time::now();
		pwm_pub.publish(pwm_msg);
	
		loop_rate.sleep();
		ros::spinOnce();
	}

	return 0;
}
	
		

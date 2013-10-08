#include <ros/ros.h> // needs rosccp dep in the Manifest
#include <HandFollow/vw.h> // needs HandFollow dep in the Manifest. Allows to use the vw msg defined in the HandFollow package
#include <differential_drive/PWM.h> // needs differential_drive in the Manifest
#include <cmath> // Has some cpp math functions than can be used for the PID controller

float Ref1=0, Ref2=0;

const float L=20,R1=5,R2=5; 				// values in cm

/* PID controller values */

float P1=1.0,P2=1.0,I1=0.0,I2=0.0,D1=0.0,D2=0.0; // This values should be allowed to change after receiving a debug msg
float cum1=0.0,cum2=0.0,diff1=0.0,diff2=0.0;

/* this message will store the PWM values to be sent to the Arduino. It should be updated on the PID function and sent on the main loop*/
differential_drive::PWM msg; 

/* This function is called when the node receives a message from the /ControlMux/vw topic and converts the v and w values into the references for each motor. These references should be in m/s or cm/s or equivalent */
void RefConverter(const HandFollow::vw::ConstPtr msg){ 	

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
int PID_control(float P,float I,float D,float integrator_sum, float diferentiator_val,float Ref,float vel){
	
	float error=Ref-vel;
	
	
	return round(P*error);
	

}

int main(int argc, char ** argv){
	ros::init(argc,argv, "MotorControl"); //initialise ros and the arguments
	ros::NodeHandle n;			// Node handler 
	ros::Subscriber vw; 			
	ros::Publisher PWM;
	
	
	/* Loop rate of 100Hz to comply with the encoders update frequency */
	ros::Rate loop_rate(100);
	ROS_INFO("Started the Motor Control Node");
	
	vw = n.subscribe("/ControlMux/vw",1,RefConverter);
	PWM =n.advertise<differential_drive::PWM>("/motion/PWM",1);
	
	/* Main loop */
	while (ros::ok()){
	
		msg.PWM1=PID_control(P1,I1,D1,cum1,diff1,Ref1,0);
		msg.PWM2=PID_control(P2,I2,D2,cum2,diff2,Ref2,0);
		PWM.publish(msg);
	
		loop_rate.sleep();
		ros::spinOnce();
	}

	return 0;
}
	
		

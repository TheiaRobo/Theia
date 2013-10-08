#include <ros/ros.h>
#include <HandFollow/vw.h>
#include <differential_drive/PWM.h>

float Ref1=0, Ref2=0; // 

const float L=20,R1=5,R2=5; // values in cm


void RefConverter(const HandFollow::vw::ConstPtr msg){ /* PID Control based on reference values obtained from the vw message */
	
	float angv, linv;
	
	linv=msg->v;
	angv=msg->w;
	
	Ref1=(linv+angv*L)/R1; // positive w turns left, ref1 for left wheel.
	Ref2=(linv-angv*L)/R2;
	
	if (Ref1 >255)
		Ref1=255;
		
	if (Ref1 <-255)
		Ref1=-255;
		
	if (Ref2 >255)
		Ref2=255;
		
	if (Ref2 <-255)
		Ref2=-255;
		
		

}

int main(int argc, char ** argv){
	ros::init(argc,argv, "MotorControl"); //initialse ros and the arguments
	ros::NodeHandle n;			// Node handler 
	ros::Subscriber vw; 
	ros::Publisher PWM;
	differential_drive::PWM msg;
	
	
	ros::Rate loop_rate(100);
	ROS_INFO("Started the Motorcontrol");
	
	vw = n.subscribe("/ControlMux/vw",1,RefConverter);
	PWM =n.advertise<differential_drive::PWM>("/motion/PWM",1);
	
	while (ros::ok()){
	
		msg.PWM1=Ref1;
		msg.PWM2=Ref2;
		PWM.publish(msg);
	
		loop_rate.sleep();
		ros::spinOnce();
	}

	return 0;
}
	
		

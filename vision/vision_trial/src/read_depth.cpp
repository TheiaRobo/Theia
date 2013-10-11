#include "ros/ros.h"
#include "stereo_msgs/DisparityImage.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cvwimage.h>
#include <stdlib.h>
#include <cmath>
#include <robot_messages/coords.h>

namespace enc = sensor_msgs::image_encodings;

image_transport::Publisher pub;
ros::Publisher pos_pub;

	
float * closest(cv::Mat * I){ // identifies the patch that has the largest disparity value and returns it's value and center pixel's coordinates: (x,y,avg_disp)
	
	int rows;
	int cols;
	int i,j;
	float max=0, *retval;
	uchar * p;
	cv::Size size;
	cv::Point center(0,0);
	cv::Scalar color(255,0,0);
	cv::Mat blurred;
	
	rows=I->rows;
	cols=I->cols;
	
	//patch size
	size.width=55;
	size.height=55;
	
	retval = new float(3);
	
	cv::GaussianBlur(*I,blurred,size,20,20); // blurring the image before processing will (hopefully) reduce noise's effects
	
	for(i=110;i<blurred.rows-120;i++){
		p=blurred.ptr<uchar>(i);
		
		for(j=199;j<blurred.cols-200;j++){
			if(p[j]>max){
				max=p[j];
				center.x=j;
				center.y=i;
			}
		}
	}
	
	
	
	
				
	retval[0]=center.x;
	retval[1]=center.y;
	retval[2]=max;
	
	
	cv::circle(*I,center,60*(max/255),color,5);
	
	cv::waitKey(1); //required to display image
	return retval;
	
	
}

void get_pts(const stereo_msgs::DisparityImage::ConstPtr &msg){

	cv::Mat I;
	cv_bridge::CvImagePtr cv_ptr;
	sensor_msgs::ImagePtr img;
	robot_messages::coords crd;

	float * info=NULL;
	
	
	try{
	//Always copy, returning a mutable CvImage
		cv_ptr = cv_bridge::toCvCopy(msg->image,enc::MONO8);
	}
	catch (cv_bridge::Exception& e){
	//if there is an error during conversion, display it
		ROS_ERROR("tutorialROSOpenCV::main.cpp::cv_bridge exception: %s", e.what());
		return;
	}
	
	I=cv_ptr->image;
	info=closest(&I);
	
	
	img = cv_ptr->toImageMsg();
	
	crd.x=info[0];
	crd.y=info[1];
	crd.z=msg->f*msg->T/info[2]*100; // approx distance in cm. Infinity values SHOULD be expected
		
	pos_pub.publish(crd);
	pub.publish(img);
	
	delete info;	
	
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "Read_Depth");
	ros::NodeHandle n;
	image_transport::ImageTransport it(n); // allows to create image publishers	
	ros::Subscriber img;
	
	
	ros::Rate loop_rate(30);

	ROS_INFO("Started the depth node");	
	
	pub=it.advertise("/read_depth/estimate",1);
	img=n.subscribe("/camera/depth_registered/disparity",1,get_pts);
	pos_pub=n.advertise<robot_messages::coords>("/read_depth/xyz",1);
	
	while(ros::ok()){

		loop_rate.sleep();
		ros::spinOnce();

	}

	
	return 0;
}

#include <iostream>
#include <string>

// ROS libs
#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

// ROS messages
#include <sensor_msgs/Image.h>

// ROS to / from OpenCV bridge
#include <cv_bridge/cv_bridge.h>

#define NODE_NAME "vision_object_snap"
#define TOPIC_IN "/camera/rgb/image_mono"

using namespace std;

cv::Mat image;
bool imageIsSet;
ros::Subscriber imageSub;

void imageCallback(const sensor_msgs::ImageConstPtr & rosMsgPtr);
void imageSave(string & fileName);

void imageCallback(const sensor_msgs::ImageConstPtr & rosMsgPtr){
	ROS_INFO("Image Callback");
	ROS_INFO("- Start");

	cv_bridge::CvImagePtr imgPtr;
	imgPtr = cv_bridge::toCvCopy(rosMsgPtr, "mono8");

	image = imgPtr->image;
	imageIsSet = true;

	ROS_INFO("- End");
}

void imageSave(string & fileName){
	if(!imageIsSet){
		ROS_INFO("Error: Image not set yet");
		return;
	}

	try{
		imwrite(fileName, image);
	}catch(runtime_error & ex){
		ROS_INFO("Error: Could not save image");
	}
}

int main(int argc, char ** argv){
	ros::init(argc, argv, NODE_NAME);
	ros::NodeHandle node;

	imageIsSet = false;
	imageSub = node.subscribe(TOPIC_IN, 1, imageCallback);

	cout << "Welcome to Snapper" << endl;
	cout << endl;

	do{
		cout << "Enter Filename: ";
		
		string fileName;
		cin >> fileName;

		// update image
		ros::spinOnce();

		// save image
		imageSave(fileName);
	}while(true);
}
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
#define TOPIC_COLOR_IN "/camera/rgb/image_rect"
#define TOPIC_DEPTH_IN "/camera/depth/image_rect"

using namespace std;
using namespace sensor_msgs;

cv::Mat colorImage;
cv::Mat depthImage;
bool colorImageSet;
bool depthImageSet;
ros::Subscriber colorImageSub;
ros::Subscriber depthImageSub;

void colorImageCallback(const ImageConstPtr & rosMsgPtr){
	cout << "Running " << __FUNCTION__ << endl;

	cv_bridge::CvImagePtr imgPtr;
	imgPtr = cv_bridge::toCvCopy(rosMsgPtr, "mono8");

	colorImage = imgPtr->image;
	colorImageSet = true;
}

void depthImageCallback(const ImageConstPtr & rosMsgPtr){
	cout << "Running " << __FUNCTION__ << endl;

	cv_bridge::CvImagePtr imgPtr;
	imgPtr = cv_bridge::toCvCopy(rosMsgPtr);

	// convert from floats to three 8 bit channels
	imgPtr->image.convertTo(depthImage, CV_8UC3, 255);
	depthImageSet = true;
}

void imageSave(string & fileName){
	if(!colorImageSet || !depthImageSet){
		cout << "Error in " << __FUNCTION__ << endl;
		cout << "Images not set" << endl;
		return;
	}

	try{
		cv::imwrite(fileName + "_color.png", colorImage);
		cv::imwrite(fileName + "_depth.png", depthImage);
	}catch(runtime_error & ex){
		cout << "Error in " << __FUNCTION__ << endl;
		cout << "Could not write images" << endl;
		return;
	}

	colorImageSet = false;
	depthImageSet = false;
}

/**
* This is a small tool for capturing color and deth images
* of an object at the same time.
* It was used for capturing training images of all objects.
*/
int main(int argc, char ** argv){
	ros::init(argc, argv, NODE_NAME);
	ros::NodeHandle node;

	colorImageSet = false;
	depthImageSet = false;

	colorImageSub = node.subscribe(TOPIC_COLOR_IN, 1, colorImageCallback);
	depthImageSub = node.subscribe(TOPIC_DEPTH_IN, 1, depthImageCallback);

	cout << "Welcome to Snapper" << endl;
	cout << endl;

	do{
		cout << "Enter Filename: ";
		
		string fileName;
		cin >> fileName;

		do{
			ros::spinOnce();
		}while(!colorImageSet || !depthImageSet);

		// save image
		imageSave(fileName);
	}while(true);
}
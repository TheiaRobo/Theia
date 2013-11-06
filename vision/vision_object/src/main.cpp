#include <iostream>
#include <string>
#include <vector>

// ROS
#include <ros/ros.h>

// ROS messages
#include <std_msgs/Empty.h>
#include <sensor_msgs/Image.h>

// ROS to / from OpenCV bridge
#include <cv_bridge/cv_bridge.h>

// Sub modules
#include "recog.h"
#include "train.h"

#define NODE_NAME "vision_object"
#define TOPIC_IN "/camera/rgb/image_mono"
#define TOPIC_TRAIN "/vision/object/train"

using namespace std;

void imageCallback(const sensor_msgs::ImageConstPtr & rosMsgPtr);
void trainCallback(const std_msgs::EmptyConstPtr & rosMsgPtr);
int trainInit();

vector<ObjectTrainData_t> trainDataVect;
ros::Subscriber imageSub;
ros::Subscriber trainSub;

void imageCallback(const sensor_msgs::ImageConstPtr & rosMsgPtr){
	cout << "Image Callback" << endl;
	cout << " Start" << endl;

	cv_bridge::CvImagePtr imagePtr;
	imagePtr = cv_bridge::toCvCopy(rosMsgPtr, "mono8");

	recog(
		imagePtr->image,
		trainDataVect
	);

	cout << " End" << endl;
}

void trainCallback(const std_msgs::EmptyConstPtr & rosMsgPtr){
	cout << "Training Callback" << endl;
	cout << " Start" << endl;
	
	trainInit();

	cout << " End" << endl;
}

int trainInit(){
	ObjectTrainConfig_t trainConfig;
	
	ros::param::getCached(
		"~config/path",
		trainConfig.path
	);

	ros::param::getCached(
		"~config/surfMinHessian",
		trainConfig.surfMinHessian
	);

	int errorCode;
	errorCode = train(
		trainConfig,
		trainDataVect
	);

	if(errorCode){
		cout << "Error: Training failed" << endl;
		return errorCode;
	}

	return 0;
}

int main(int argc, char ** argv){
	/**
	* Init ROS
	*/
	ros::init(argc, argv, NODE_NAME);
	ros::NodeHandle node;

	imageSub = node.subscribe(TOPIC_IN, 1, imageCallback);
	trainSub = node.subscribe(TOPIC_TRAIN, 1, trainCallback);

	/**
	* Init training data
	*/
	int errorCode;
	errorCode = trainInit();

	if(errorCode){
		cout << "Error: Could not init" << endl;
		return errorCode;
	}

	/**
	* Run object recognition
	*/
	ros::spin();

	return 0;
}
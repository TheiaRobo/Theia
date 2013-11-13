#include <iostream>
#include <string>
#include <vector>

// ROS
#include <ros/ros.h>
#include <vision/array.h>
#include <vision/image.h>

// ROS messages
#include <sensor_msgs/Image.h>
#include <std_msgs/Empty.h>

// ROS to / from OpenCV bridge
#include <cv_bridge/cv_bridge.h>

// Sub modules
#include "recog.h"
#include "train.h"

#define NODE_NAME "vision_object"
#define TOPIC_IN "/camera/rgb/image_mono"
#define TOPIC_TRAIN "/vision/object/train"

using namespace std;

TheiaImageContext imageContext;
Array<ObjectTrainData_t> trainDataArr;
ros::Subscriber imageSub;
ros::Subscriber trainSub;

int trainInit(){
	string path;
	ros::param::getCached(
		"~config/path",
		path
	);

	int surfMinHessian;
	ros::param::getCached(
		"~config/surfMinHessian",
		surfMinHessian
	);

	int errorCode;
	errorCode = theiaImageCreateContext(
		surfMinHessian,
		imageContext
	);

	if(errorCode){
		cout << "Error: Could not create image context" << endl;
		return errorCode;
	}

	ObjectTrainConfig_t trainConfig;
	trainConfig.path = path;
	trainConfig.imageContext = imageContext;

	vector<ObjectTrainData_t> trainDataVect;
	errorCode = train(
		trainConfig,
		trainDataVect
	);

	if(errorCode){
		cout << "Error: Training failed" << endl;
		return errorCode;
	}

	// to array
	trainDataArr = Array<ObjectTrainData_t>(trainDataVect);

	return 0;
}

void imageCallback(const sensor_msgs::ImageConstPtr & rosMsgPtr){
	cout << "Image Callback" << endl;
	cout << " Start" << endl;

	cv_bridge::CvImagePtr imagePtr;
	imagePtr = cv_bridge::toCvCopy(rosMsgPtr, "mono8");

	TheiaImageData imageData;
	imageData.image = imagePtr->image;

	double minScore;
	ros::param::getCached(
		"~config/recogMinScore",
		minScore
	);

	ObjectRecogContext recogContext;
	recogContext.minScore = minScore;
	recogContext.imageContextPtr = &imageContext;

	recog(imageData, trainDataArr, recogContext);

	cout << " End" << endl;
}

void trainCallback(const std_msgs::EmptyConstPtr & rosMsgPtr){
	cout << "Training Callback" << endl;
	cout << " Start" << endl;
	
	trainInit();

	cout << " End" << endl;
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
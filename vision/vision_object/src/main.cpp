#include <iostream>
#include <string>
#include <vector>
#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Empty.h>

#include "object.h"

#define NODE_NAME "vision_object"
#define TOPIC_IN "/camera/rgb/image_mono"

using namespace std;

Config config;
Context context(config);
vector<Object> objectVect;
ros::Subscriber colorImageSub;

int init(){
	int errorCode = 0;

	/**
	* TODO
	* Copy configuration options from parameter server into
	* config class
	*/
	config = Config();
	config.colorImage.minHessian = 400;

	context = Context(config);

	return errorCode;
}

int match(const ObjectData & inSampleData){
	int errorCode = 0;

	size_t numbObjects = objectVect.size();
	vector<ObjectDataResult> resultVect(numbObjects);

	for(size_t i = 0; i < numbObjects; i++){
		ObjectDataResult result;

		errorCode = objectVect[i].match(inSampleData, context, result);
		if(errorCode) return errorCode;

		cout << "Object " << i << endl;
		cout << " Score: " << result.colorImage.meanSquareError << endl;

		resultVect.push_back(result);
	}

	return errorCode;
}

int train(){
	int errorCode = 0;
	string trainPath = "/home/amotta/Documents/Theia/vision/vision_object/train";

	errorCode = Object::find(trainPath, objectVect);
	if(errorCode) return errorCode;

	size_t numbObjects = objectVect.size();
	for(size_t i = 0; i < numbObjects; i++){
		Object & object = objectVect[i];
		size_t numbData = object.objectDataVect.size();

		cout << "Object " << i << endl;
		cout << " Name: " << object.name << endl;
		cout << " # Data: " << numbData << endl;
		
		cout << " Train .." << endl;
		object.train(context);

		cout << " Show results .." << endl;
		for(size_t j = 0; j < numbData; j++){
			ObjectData & data = object.objectDataVect[j];
			data.colorImage.show();
		}

	}

	return errorCode;
}

void colorImageCallback(const sensor_msgs::ImageConstPtr & rosMsgPtr){
	int errorCode = 0;

	cv_bridge::CvImagePtr imagePtr;
	imagePtr = cv_bridge::toCvCopy(rosMsgPtr, "mono8");

	ObjectData sampleData;
	ColorImageData & colorImageData = sampleData.colorImage;
	ColorImageContext & colorImageContext = context.colorImage;

	errorCode = colorImageData.train(imagePtr->image, colorImageContext);
	if(errorCode){
		cout << "Error in " << __FUNCTION__ << endl;
		cout << "Could not train color image" << endl;
		return;
	}

	colorImageData.show();

	errorCode = match(sampleData);
	if(errorCode){
		cout << "Error in " << __FUNCTION__ << endl;
		cout << "Could not match object data" << endl;
		return;
	}
}

int main(int argc, char ** argv){
	int errorCode = 0;

	ros::init(argc, argv, NODE_NAME);
	ros::NodeHandle node;

	colorImageSub = node.subscribe(TOPIC_IN, 1, colorImageCallback);

	errorCode = init();
	if(errorCode) return errorCode;

	errorCode = train();
	if(errorCode) return errorCode;

/*
	errorCode = match();
	if(errorCode) return;
*/

	ros::spin();

	return errorCode;
}
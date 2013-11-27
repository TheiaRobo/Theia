#include <iostream>
#include <string>
#include <utility>
#include <vector>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Empty.h>
#include <vision_object/Object.h>
#include <vision_plane/Candidate.h>
#include <vision_plane/Candidates.h>

#include "object.h"

#define NODE_NAME "vision_object"
#define TOPIC_IN_CAND "/vision/plane/cand"
#define TOPIC_IN_COLOR "/camera/rgb/image_rect"
#define TOPIC_IN_DEPTH "/camera/depth/image_rect"
#define TOPIC_OUT_OBJECT "/vision/object"

using namespace std;
using namespace message_filters;
using namespace vision_plane;
using namespace sensor_msgs;

Config config;
Context context(config);
vector<Candidate> candVect;
vector<Object> objectVect;
ObjectData sampleData;
bool candVectReady;
bool colorImageReady;
bool depthImageReady;
ros::Subscriber candSub;
ros::Subscriber colorImageSub;
ros::Subscriber depthImageSub;
ros::Publisher objectPub;

int init(){
	int errorCode = 0;

	config = Config();
	ros::param::getCached(
		"~config/path",
		config.path
	);
	ros::param::getCached(
		"~config/colorImage/minHessian",
		config.colorImage.minHessian
	);
	ros::param::getCached(
		"~config/colorImage/maxMeanSquareError",
		config.colorImage.maxMeanSquareError
	);
	ros::param::getCached(
		"~config/colorImage/numbMatchesHomography",
		config.colorImage.numbMatchesHomography
	);
	ros::param::getCached(
		"~config/depthImage/blurring",
		config.depthImage.blurring
	);
	ros::param::getCached(
		"~config/depthImage/cannyLevelOne",
		config.depthImage.cannyLevelOne
	);
	ros::param::getCached(
		"~config/depthImage/cannyLevelTwo",
		config.depthImage.cannyLevelTwo
	);

	context = Context(config);

	return errorCode;
}

int publishResults(
	const vector< pair<Object, ObjectDataResult> > & inResults
){
	int errorCode = 0;

	size_t numbResults = inResults.size();
	for(size_t i = 0; i < numbResults; i++){
		const Object & object = inResults[i].first;
		const ObjectDataResult & result = inResults[i].second;

		vision_object::Object msg;
		msg.objectName = object.name;
		msg.objectAngle = result.angle;

		objectPub.publish(msg);
	}
	
	return errorCode;
}

int match(){
	int errorCode = 0;

	size_t numbObjects = objectVect.size();
	vector< pair<Object, ObjectDataResult> > resultVect;

	for(size_t i = 0; i < numbObjects; i++){
		ObjectDataResult result;

		Object & object = objectVect[i];
		errorCode = object.match(sampleData, context, result);
		if(errorCode) return errorCode;

		cout << "Object: " << object.name << endl;
		cout << "Score: " << result.colorImage.meanSquareError << endl;

		if(result.isGoodEnough(context)){
			resultVect.push_back(make_pair(object, result));
		}
	}

	errorCode = publishResults(resultVect);
	if(errorCode){
		cout << "Error in " << __FUNCTION__ << endl;
		cout << "Could not publish results" << endl;
		return -1;
	}

	return errorCode;
}

int train(){
	int errorCode = 0;

	errorCode = Object::find(context.path, objectVect);
	if(errorCode) return errorCode;

	size_t numbObjects = objectVect.size();
	for(size_t i = 0; i < numbObjects; i++){
		Object & object = objectVect[i];
		size_t numbData = object.objectDataVect.size();

		cout << "Object " << i << endl;
		cout << " Name: " << object.name << endl;
		cout << " # Data: " << numbData << endl;
		
		cout << " Train .." << endl;

		errorCode = object.train(context);
		if(errorCode) return errorCode;

		cout << " Show results .." << endl;
		for(size_t j = 0; j < numbData; j++){
			ObjectData & data = object.objectDataVect[j];
			data.colorImage.showKeypoints();
/*
			data.depthImage.show();
*/
		}

	}

	return errorCode;
}

int tryToMatch(){
	int errorCode = 0;

	if(!candVectReady) return 0;
	if(!colorImageReady) return 0;
	if(!depthImageReady) return 0;

	errorCode = match();
	if(errorCode) return errorCode;

	candVectReady = false;
	colorImageReady = false;
	colorImageReady = false;

	return errorCode;
}

void candCallback(const CandidatesConstPtr & candsMsgPtr){
	int errorCode = 0;

	const vector<Candidate> & candVect = candsMsgPtr->candidates;
	size_t numbCands = candVect.size();

	for(size_t i = 0; i < numbCands; i++){
		const Candidate & cand = candVect[i];
		cout << "Candidate " << i << endl;
		cout << "Min Latitude "<< cand.minLatitude << endl;
		cout << "Max Latitude "<< cand.maxLatitude << endl;
		cout << "Min Longitude "<< cand.minLongitude << endl;
		cout << "Max Longitude "<< cand.maxLongitude << endl;
	}

	// TODO: enable
	// candVectReady = true;

	errorCode = tryToMatch();
	if(errorCode){
		cout << "Error in " << __FUNCTION__ << endl;
		cout << "Matching failed" << endl;
		return;
	}
}

void colorCallback(const ImageConstPtr & colorMsgPtr){
	int errorCode = 0;

	cv_bridge::CvImagePtr imagePtr;
	imagePtr = cv_bridge::toCvCopy(colorMsgPtr);

	cv::Mat & image = imagePtr->image; 
	if(!image.data){
		cout << "Error in " << __FUNCTION__ << endl;
		cout << "Could not convert image to OpenCV data" << endl;
		return;
	}

	ColorImageData & imageData = sampleData.colorImage;
	ColorImageContext & imageContext = context.colorImage;
	errorCode = imageData.train(image, imageContext);
	if(errorCode){
		cout << "Error in " << __FUNCTION__ << endl;
		cout << "Could not train color image" << endl;
		return;
	}

	colorImageReady = true;

	errorCode = tryToMatch();
	if(errorCode){
		cout << "Error in " << __FUNCTION__ << endl;
		cout << "Matching failed" << endl;
		return;
	}
}

void depthCallback(const ImageConstPtr & depthMsgPtr){
	int errorCode = 0;

	cv_bridge::CvImagePtr imagePtr;
	imagePtr = cv_bridge::toCvCopy(depthMsgPtr);

	cv::Mat & floatImage = imagePtr->image; 
	if(!floatImage.data){
		cout << "Error in " << __FUNCTION__ << endl;
		cout << "Could not convert image to OpenCV data" << endl;
		return;
	}

	// convert to grayscale
	cv::Mat image;
	imagePtr->image.convertTo(image, CV_8UC1, 255);

	DepthImageData & imageData = sampleData.depthImage;
	DepthImageContext & imageContext = context.depthImage;
	errorCode = imageData.train(image, imageContext);
	if(errorCode){
		cout << "Error in " << __FUNCTION__ << endl;
		cout << "Could not train depth image" << endl;
		return;
	}

	depthImageReady = true;

	errorCode = tryToMatch();
	if(errorCode){
		cout << "Error in " << __FUNCTION__ << endl;
		cout << "Matching failed" << endl;
		return;
	}
}

int main(int argc, char ** argv){
	int errorCode = 0;

	ros::init(argc, argv, NODE_NAME);
	ros::NodeHandle node;

	candSub = node.subscribe(TOPIC_IN_CAND, 5, candCallback);
	colorImageSub = node.subscribe(TOPIC_IN_COLOR, 1, colorCallback);
/*
	depthImageSub = node.subscribe(TOPIC_IN_DEPTH, 1, depthCallback);
*/
	objectPub = node.advertise<vision_object::Object>(
		TOPIC_OUT_OBJECT, 1
	);

	candVectReady = false;
	colorImageReady = false;
	depthImageReady = false;

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
#include <algorithm>
#include <iostream>
#include <string>
#include <utility>
#include <vector>
#include <cv_bridge/cv_bridge.h>
<<<<<<< HEAD
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
=======
>>>>>>> vision-devel
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <vision_object/Object.h>
#include <vision_plane/Candidate.h>
#include <vision_plane/Candidates.h>

#include "candidate.h"
#include "config.h"
#include "context.h"
#include "object.h"

#define NODE_NAME "vision_object"
#define TOPIC_IN_CAND "/vision/plane/cand"
#define TOPIC_IN_COLOR "/camera/rgb/image_rect"
#define TOPIC_OUT_OBJECT "/vision/object"

using namespace std;
using namespace vision_plane;
using namespace sensor_msgs;

Config config;
CameraConfig cameraConfig;
Context context(config);
bool candValid;
vector<Candidate> candVect;
vector<Object> objectVect;
ObjectData sampleData;
bool candVectReady;
bool colorImageReady;
ros::Subscriber candSub;
ros::Subscriber colorImageSub;
ros::Publisher objectPub;

int init(){
	int errorCode = 0;

	config = Config();
	context = Context(config);

	return errorCode;
}

bool compareResultPairs(
	const pair<Object, ObjectDataResult> & inOne,
	const pair<Object, ObjectDataResult> & inTwo
){
	return inOne.second.isBetterThan(inTwo.second);
}

int publishResults(
	const vector< pair<Object, ObjectDataResult> > & inResults
){
	int errorCode = 0;
	
	if(inResults.empty()) return errorCode;
	if(validCandVect.empty()) return errorCode;
	
	// sort results
	vector< pair<Object, ObjectDataResult> > workingVect(inResults);
	sort(workingVect.begin(), workingVect.end(), compareResultPairs);
	if(inResults.empty()) return errorCode;

	/**
	* TODO
	* Clean up
	*/
	size_t numbCands = candVect.size();
	if(!numbCands) return errorCode;

	Candidate * candPtr = NULL;
	for(size_t i = 0; i < numbCands; i++){
		if(candCheckIfValid(candVect[i], context.camera)){
			candPtr = &candVect[i];
		}
	}

	if(!candPtr) return errorCode;

	double box[3][2];
	candToBox(*candPtr, context.camera, box);

	// sort results
	vector< pair<Object, ObjectDataResult> > workingVect(inResults);
	sort(workingVect.begin(), workingVect.end(), compareResultPairs);

	const Object & object = inResults[0].first;
	const ObjectDataResult & result = inResults[0].second;

	cout << "Best Object: " << object.name;
	
	vision_object::Object msg;
	msg.objectName = object.name;
	msg.objectAngle = result.angle;
	msg.distX = (box[0][0] + box[0][1]) / 2;
	msg.distY = (box[1][0] + box[1][1]) / 2;

	objectPub.publish(msg);
	return errorCode;
}

int match(){
	int errorCode = 0;
	
	if(!candValid){
		cout << "No valid object found" << endl;
		return errorCode;
	}
	
	cout << "Valid object found" << endl;

/*
	errorCode = candDebug();
	if(errorCode){
		cout << "Error in " << __FUNCTION__ << endl;
		cout << "Could not show candidates" << endl;
		return errorCode;
	}
*/

	size_t numbObjects = objectVect.size();
	pair<Object, ObjectDataResult> bestResult;
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
		}

	}

	return errorCode;
}

int tryToMatch(){
	int errorCode = 0;

	if(!candVectReady) return 0;
	if(!colorImageReady) return 0;

	errorCode = match();
	if(errorCode) return errorCode;

	candVectReady = false;
	colorImageReady = false;

	return errorCode;
}

int candDebug(){
	int errorCode = 0;

	if(!candVectReady) return errorCode;
	if(!colorImageReady) return errorCode;

/*
	cv::Mat image;
	errorCode = candShow(
		candVect,
		sampleData.colorImage.image,
		image
	);
	if(errorCode) return errorCode;

	cv::imshow("Candidates", image);
	cv::waitKey(0);
*/

	return errorCode;
}

void candCallback(const CandidatesConstPtr & candsMsgPtr){
	int errorCode = 0;

	// copy candidates
	candVect = vector<Candidate>(candsMsgPtr->candidates);
	candVectReady = true;

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

int main(int argc, char ** argv){
	int errorCode = 0;

	candValid = false;
	candVectReady = false;
	colorImageReady = false;

	ros::init(argc, argv, NODE_NAME);
	ros::NodeHandle node;

	// subscribers
	candSub = node.subscribe(TOPIC_IN_CAND, 5, candCallback);
	colorImageSub = node.subscribe(TOPIC_IN_COLOR, 1, colorCallback);

	// publisher
	objectPub = node.advertise<vision_object::Object>(
		TOPIC_OUT_OBJECT, 1
	);
	
	// main

	errorCode = init();
	if(errorCode) return errorCode;

	errorCode = train();
	if(errorCode) return errorCode;

	ros::spin();

	return errorCode;
}

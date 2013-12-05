#include <algorithm>
#include <iostream>
#include <string>
#include <utility>
#include <vector>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <vision_object/Object.h>
#include <vision_plane/Box.h>
#include <vision_plane/Boxes.h>

#include "candidate.h"
#include "config.h"
#include "context.h"
#include "object.h"

#define NODE_NAME "vision_object"
#define TOPIC_IN_BOX "/vision/plane/box"
#define TOPIC_IN_COLOR "/camera/rgb/image_rect"
#define TOPIC_OUT_OBJECT "/vision/object"

using namespace std;
using namespace vision_plane;
using namespace sensor_msgs;

Config config;
Context context(config);
bool candValid;
vector<Candidate> candVect;
vector<Object> objectVect;
ObjectData sampleData;
bool candVectReady;
bool colorImageReady;
ros::Subscriber boxSub;
ros::Subscriber colorImageSub;
ros::Publisher objectPub;

int init(){
	int errorCode = 0;

	config = Config();
	errorCode = configBuild(config);
	if(errorCode){
		cout << "Error in " << __FUNCTION__ << endl;
		cout << "Could not load configuration" << endl;
		return errorCode;
	}

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

	// sort results
	vector< pair<Object, ObjectDataResult> > workingVect(inResults);
	sort(workingVect.begin(), workingVect.end(), compareResultPairs);

	const Object & object = inResults[0].first;
	const ObjectDataResult & result = inResults[0].second;

	cout << "Best Object: " << object.name;
	
	vision_object::Object msg;
	msg.objectName = object.name;
	msg.objectAngle = result.angle;
/*
	msg.distX = (box[0][0] + box[0][1]) / 2;
	msg.distY = (box[1][0] + box[1][1]) / 2;
*/
	objectPub.publish(msg);

	return errorCode;
}

int match(){
	int errorCode = 0;

	size_t numbObjects = objectVect.size();
	if(!numbObjects) return errorCode;

	size_t numbCands = candVect.size();
	if(!numbCands) return errorCode;

	vector<Candidate> validCands;
	errorCode = candFilterValid(candVect, validCands);
	if(errorCode){
		cout << "Error in " << __FUNCTION__ << endl;
		cout << "Could not filter valid candidates" << endl;
		return errorCode;
	}

	size_t numbValidCands = validCands.size();
	if(!numbValidCands){
		cout << "No valid object candidates" << endl;
		return errorCode;
	}

	cout << "Valid candidates" << endl;
	for(size_t i = 0; i < numbValidCands; i++){
		candPrint(validCands[i]);
	}

	/**
	* CLEAN UP NEEDED
	*/
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

		errorCode = object.train(context);
		if(errorCode){
			cout << "Error in " << __FUNCTION__ << endl;
			cout << "Could not train " << object.name << endl;
			return errorCode;
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

void boxCallback(const BoxesConstPtr & boxesMsgPtr){
	int errorCode = 0;

	const vector<Box> & boxVect = boxesMsgPtr->boxes;

	size_t numbBoxes = boxVect.size();
	if(!numbBoxes) return;

	candVect.clear();
	for(size_t i = 0; i < numbBoxes; i++){
		const Box & box = boxVect[i];

		Candidate cand;
		errorCode = candFromBox(box, context.camera, cand);
		if(errorCode){
			cout << "Error in " << __FUNCTION__ << endl;
			cout << "Conversion from box to candidate failed" << endl;
			return;	
		}

		candVect.push_back(cand);
	}

	size_t numbCands = candVect.size();
	candVectReady = (numbCands > 0);

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
	boxSub = node.subscribe(TOPIC_IN_BOX, 5, boxCallback);
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
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
#define TOPIC_IN_COLOR "/camera/rgb/image_rect_color"
#define TOPIC_OUT_OBJECT "/vision/object"

using namespace std;
using namespace vision_plane;
using namespace sensor_msgs;

Config config;
Context context(config);
vector<Object> objectVect;
vector<Candidate> candVect;
vector<Candidate> validCandVect;
bool candVectReady;
cv::Mat colorImage;
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
	if(validCandVect.empty()) return errorCode;

	// sort results
	vector< pair<Object, ObjectDataResult> > workingVect(inResults);
	sort(workingVect.begin(), workingVect.end(), compareResultPairs);

	const Object & object = workingVect[0].first;
	const ObjectDataResult & result = workingVect[0].second;

	// TODO
	// improve
	const Candidate & cand = validCandVect[0];

	cout << "Best Object: " << object.name << endl;
	
	vision_object::Object msg;
	msg.objectName = object.name;
	msg.objectAngle = result.angle;
	msg.distX = (cand.robXMin + cand.robXMax) / 2;
	msg.distY = (cand.robYMin + cand.robYMax) / 2;

	// convert colorImage to image message
	cv_bridge::CvImage cvImage;
	cvImage.encoding = "bgr8";
	cvImage.image = colorImage;
	cvImage.toImageMsg(msg.image);

	objectPub.publish(msg);

	return errorCode;
}

int match(){
	int errorCode = 0;

	size_t numbObjects = objectVect.size();
	if(!numbObjects) return errorCode;

	size_t numbCands = candVect.size();
	if(!numbCands) return errorCode;

	validCandVect.clear();
	errorCode = candFilterValid(
		candVect,
		context.candidate,
		validCandVect
	);
	if(errorCode){
		cout << "Error in " << __FUNCTION__ << endl;
		cout << "Could not filter valid candidates" << endl;
		return errorCode;
	}

	size_t numbValidCands = validCandVect.size();
	if(!numbValidCands){
		cout << "No valid object candidates" << endl;
		return errorCode;
	}

	cout << "# total candidates: " << numbCands << endl;
	cout << "# valid candidates: " << numbValidCands << endl;

	pair<Object, ObjectDataResult> bestResult;
	vector< pair<Object, ObjectDataResult> > resultVect;

	for(size_t nCand = 0; nCand < numbValidCands; nCand++){
		Candidate & cand = validCandVect[nCand];

		cv::Rect rect;
		cand.toRect(context.camera, colorImage, rect);

		ObjectData sampleData;
		ColorImageData & colorData = sampleData.colorImage;

		cv::Mat candImage(colorImage, rect);
		errorCode = colorData.train(candImage, context.colorImage);
		if(errorCode){
			cout << "Error in " << __FUNCTION__ << endl;
			cout << "Could not train color image" << endl;
			return errorCode;
		}

		for(size_t nObj = 0; nObj < numbObjects; nObj++){
			Object & object = objectVect[nObj];

			ObjectDataResult result;
			errorCode = object.match(sampleData, context, result);
			if(errorCode){
				cout << "Error in " << __FUNCTION__ << endl;
				cout << "Object matching failed" << endl;
				return errorCode;
			}

			cout << "Object: " << object.name << endl;
			cout << " Color: " << result.colorImage.colorError << endl;
			cout << " Keypoint: " << result.colorImage.keypointError << endl;
			cout << " Total: " << result.colorImage.totalError << endl;

			if(result.isGoodEnough(context)){
				resultVect.push_back(make_pair(object, result));
			}
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

		Candidate cand(box, context.camera);
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
	imagePtr = cv_bridge::toCvCopy(colorMsgPtr, "bgr8");

	colorImage = imagePtr->image; 
	if(!colorImage.data){
		cout << "Error in " << __FUNCTION__ << endl;
		cout << "Could not convert image to OpenCV data" << endl;
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
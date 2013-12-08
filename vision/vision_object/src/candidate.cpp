#include <cmath>
#include <iostream>

#include "candidate.h"

using namespace std;
using namespace vision_plane;

Candidate::Candidate(){
	// nothing
}

Candidate::Candidate(
	const Box & inBox,
	const CameraContext & inContext
){
	calcRobCoordsFromBox(inBox, inContext);	
	calcCamCoordsFromBox(inBox, inContext);
}

int Candidate::calcCamCoordsFromBox(
	const Box & inBox,
	const CameraContext & inContext
){
	int errorCode = 0;

	double latMin = atan2(inBox.maxY, inBox.minZ);
	double latMax = atan2(inBox.minY, inBox.maxZ);

	double longMin;
	double longMax;
	if(inBox.maxX < 0){
		longMin = -atan2(inBox.maxX, inBox.maxZ);
		longMax = -atan2(inBox.minX, inBox.minZ);
	}else if(inBox.minX > 0){
		longMin = -atan2(inBox.maxX, inBox.minZ);
		longMax = -atan2(inBox.minX, inBox.maxZ);
	}else{
		longMin = -atan2(inBox.maxX, inBox.minZ);
		longMax = -atan2(inBox.minX, inBox.minZ);
	}

	camLatMin = latMin;
	camLatMax = latMax;
	camLongMin = longMin;
	camLongMax = longMax;

	return errorCode;
}

int Candidate::calcRobCoordsFromBox(
	const Box & inBox,
	const CameraContext & inContext
){
	int errorCode = 0;

	double offsetX = -inContext.posX;
	double offsetY = -inContext.posY;
	double offsetZ = -inContext.posZ;
	double offsetAngle = inContext.angle * M_PI / 180;
	
	double lengthBoxX = (inBox.maxZ - inBox.minZ);
	double centerBoxX = (inBox.minZ + inBox.maxZ) / 2;

	double lengthBoxZ = (inBox.maxX - inBox.minX);
	double centerBoxZ = - (inBox.minX + inBox.maxX) / 2;

	// world coordinates
	double robXCenter = offsetX;
	robXCenter += centerBoxX * cos(offsetAngle);
	robXCenter += centerBoxZ * sin(offsetAngle);
	robXMin = robXCenter - lengthBoxX / 2;
	robXMax = robXCenter + lengthBoxX / 2;

	robYMin = offsetY - inBox.maxX;
	robYMax = offsetY - inBox.minX;

	double robZCenter = offsetZ;
	robZCenter += - centerBoxX * sin(offsetAngle);
	robZCenter += centerBoxZ * cos(offsetAngle);
	robZMin = robZCenter - lengthBoxZ / 2;
	robZMax = robZCenter + lengthBoxZ / 2;

	return errorCode;
}

/**
* TODO
* Use context in order to define valid region.
*/
bool Candidate::isValid() const {	
	// right
	if(robYMin < -0.15) return false;
	// left
	if(robYMax > +0.15) return false;
	
	return true;
}

int Candidate::print() const {
	int errorCode = 0;

	cout << "Candidate" << endl;

	cout << " xMin: " << robXMin;
	cout << " xMax: " << robXMax;
	cout << endl;

	cout << " yMin: " << robYMin;
	cout << " yMax: " << robYMax;
	cout << endl;

	cout << " zMin: " << robZMin;
	cout << " zMax: " << robZMax;
	cout << endl;

	return errorCode;
}

int Candidate::toRect(
	const CameraContext & inContext,
	const cv::Mat & inImage,
	cv::Rect & outRect
) const {
	int errorCode = 0;

	double camFOVLat = inContext.fovLat * M_PI / 180;
	double camFOVLong = inContext.fovLong * M_PI / 180;

	size_t imageCols = inImage.cols;
	size_t imageRows = inImage.rows;

	double tanDepthX = (double) imageCols / (2 * tan(camFOVLong / 2));
	double tanDepthY = (double) imageRows / (2 * tan(camFOVLat / 2));

	double minX = tanDepthX * atan(camLongMin);
	double maxX = tanDepthX * atan(camLongMax);
	double minY = tanDepthY * atan(camLatMin);
	double maxY = tanDepthY * atan(camLatMax);

	outRect = cv::Rect(
		cv::Point(minX, minY),
		cv::Point(maxX, maxY)
	);

	return errorCode;
}

int candFilterValid(
	const vector<Candidate> & inCands,
	vector<Candidate> & outCands
){
	int errorCode = 0;

	size_t numbCands = inCands.size();
	if(!numbCands) return errorCode;

	outCands.clear();
	for(size_t i = 0; i < numbCands; i++){
		if(inCands[i].isValid()){
			outCands.push_back(inCands[i]);
		}
	}

	return errorCode;
}

int candShow(
	const vector<Candidate> & inCandVect,
	const CameraContext & inContext,
	const cv::Mat & inImage,
	cv::Mat & outImage
){
	int errorCode = 0;

	size_t numbCands = inCandVect.size();
	if(!numbCands) return errorCode;

	outImage = inImage.clone();
	const cv::Scalar color(0, 255, 255);

	for(size_t i = 0; i < numbCands; i++){
		const Candidate & cand = inCandVect[i];

		cv::Rect rect;
		errorCode = cand.toRect(inContext, outImage, rect);
		if(errorCode) return errorCode;
		
		cv::rectangle(outImage, rect, color);
	}

	return errorCode;
}
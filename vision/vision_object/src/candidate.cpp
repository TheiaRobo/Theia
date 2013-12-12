#include <cmath>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>

#include "candidate.h"

using namespace std;
using namespace vision_plane;

Candidate::Candidate(){
	// nothing
}

/**
* Building an object candidate using the bounding box
* received from the object detection algorithm.
*/
Candidate::Candidate(
	const Box & inBox,
	const CameraContext & inContext
){
	calcRobCoordsFromBox(inBox, inContext);	
	calcCamCoordsFromBox(inBox, inContext);
}

/**
* Calculate latitude and longitude of the object
* relative to the camera's mounting point.
*/
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

/**
* Calculate coordinate of the bounding box around the object
* candidate in the coordinate system relative to the robot's
* center of gravity.
*/
int Candidate::calcRobCoordsFromBox(
	const Box & inBox,
	const CameraContext & inContext
){
	int errorCode = 0;

	double offsetX = inContext.posX;
	double offsetY = inContext.posY;
	double offsetZ = inContext.posZ;
	double offsetAngle = inContext.angle * M_PI / 180;
	
	double lengthBoxX = (inBox.maxZ - inBox.minZ);
	double centerBoxX = (inBox.minZ + inBox.maxZ) / 2;

	double lengthBoxZ = (inBox.maxY - inBox.minY);
	double centerBoxZ = -(inBox.minY + inBox.maxY) / 2;

	// world coordinates
	double robXCenter = offsetX;
	robXCenter += centerBoxX * cos(offsetAngle);
	robXCenter += centerBoxZ * sin(offsetAngle);
	robXMin = robXCenter - lengthBoxX / 2;
	robXMax = robXCenter + lengthBoxX / 2;

	robYMin = offsetY - inBox.maxX;
	robYMax = offsetY - inBox.minX;

	double robZCenter = offsetZ;
	robZCenter -= centerBoxX * sin(offsetAngle);
	robZCenter += centerBoxZ * cos(offsetAngle);
	robZMin = robZCenter - lengthBoxZ / 2;
	robZMax = robZCenter + lengthBoxZ / 2;

	return errorCode;
}

bool Candidate::isValid(const CandidateContext & inContext) const {	
	// too close?
	if(robXMin < inContext.minX) return false;
	// too far way?
	if(robXMax > inContext.maxX) return false;
	// too far left?
	if(robYMin < inContext.minY) return false;
	// too far right?
	if(robYMax > inContext.maxY) return false;
	// too low?
	if(robZMin < inContext.minZ) return false;
	// hovering over the floor?
	if(robZMin > 0.05) return false;
	// too high?
	if(robZMax > inContext.maxZ) return false;
	
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

/**
* Transform the latitude and longitude coordinates of the object
* candidate to a rectangle in the color image.
*/
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

	double minX = imageCols / 2 - tanDepthX * atan(camLongMin);
	double maxX = imageCols / 2 - tanDepthX * atan(camLongMax);
	double minY = imageRows / 2 + tanDepthY * atan(camLatMin);
	double maxY = imageRows / 2 + tanDepthY * atan(camLatMax);

	double centerX = (minX + maxX) / 2;
	double lengthX = (maxX - minX) / 2;
	double centerY = (minY + maxY) / 2;
	double lengthY = (maxY - minY) / 2;

	double factor = 2;
	double tempMinX = centerX - factor * lengthX;
	double tempMaxX = centerX + factor * lengthX;
	double tempMinY = centerY - factor * lengthY;
	double tempMaxY = centerY + factor * lengthY;
	
	minX = min(tempMinX, tempMaxX);
	maxX = max(tempMinX, tempMaxX);
	minY = min(tempMinY, tempMaxY);
	maxY = max(tempMinY, tempMaxY);

	if(minX < 0) minX = 0;
	if(maxX > imageCols) maxX = imageCols - 1;
	if(minY < 0) minY = 0;
	if(maxY > imageRows) maxY = imageRows - 1;
	
	outRect.x = minX;
	outRect.y = minY;
	outRect.width = maxX - minX;
	outRect.height = maxY - minY;

	return errorCode;
}

/**
* Remove all invalid object candidate from a vector.
*/
int candFilterValid(
	const vector<Candidate> & inCands,
	const CandidateContext & inContext,
	vector<Candidate> & outCands
){
	int errorCode = 0;

	size_t numbCands = inCands.size();
	if(!numbCands) return errorCode;

	outCands.clear();
	for(size_t i = 0; i < numbCands; i++){
		if(inCands[i].isValid(inContext)){
			outCands.push_back(inCands[i]);
		}
	}

	return errorCode;
}

/**
* Show a color image with superposed rectangle around
* the object candidates.
*/
int candShow(
	const vector<Candidate> & inCandVect,
	const CameraContext & inContext,
	const cv::Mat & inImage
){
	int errorCode = 0;

	size_t numbCands = inCandVect.size();
	if(!numbCands) return errorCode;

	cv::Mat image = inImage.clone();
	const cv::Scalar color(0, 255, 255);

	for(size_t i = 0; i < numbCands; i++){
		const Candidate & cand = inCandVect[i];

		cv::Rect rect;
		errorCode = cand.toRect(inContext, image, rect);
		if(errorCode) return errorCode;
		
		cv::rectangle(image, rect, color);
	}

	cv::imshow("Candidates", image);
	cv::waitKey(0);

	return errorCode;
}

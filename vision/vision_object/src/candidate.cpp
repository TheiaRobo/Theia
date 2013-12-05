#include <cmath>
#include <iostream>

#include "candidate.h"

using namespace std;
using namespace vision_plane;

int candFilterValid(
	const vector<Candidate> & inCands,
	vector<Candidate> & outCands
){
	int errorCode = 0;

	size_t numbCands = inCands.size();
	if(!numbCands) return errorCode;

	outCands.clear();
	for(size_t i = 0; i < numbCands; i++){
		if(candIsValid(inCands[i])){
			outCands.push_back(inCands[i]);
		}
	}

	return errorCode;
}

int candFromBox(
	const Box & inBox,
	const CameraContext & inContext,
	Candidate & outCand
){
	int errorCode = 0;

	double camPosX = -0.10;
	double camPosY = +0.00;
	double camPosZ = +0.35;

	// TODO
	// Take value from context
	double camTheta = -0.1;
	
	double lengthBoxX = (inBox.maxZ - inBox.minZ);
	double centerBoxX = (inBox.minZ + inBox.maxZ) / 2;

	double lengthBoxZ = (inBox.maxX - inBox.minX);
	double centerBoxZ = - (inBox.minX + inBox.maxX) / 2;

	Candidate cand;

	// world coordinates
	double robXCenter = camPosX + centerBoxX * cos(camTheta) + centerBoxZ * sin(camTheta);
	cand.robXMin = robXCenter - lengthBoxX / 2;
	cand.robXMax = robXCenter + lengthBoxX / 2;

	cand.robYMin = camPosY - inBox.maxX;
	cand.robYMax = camPosY - inBox.minX;

	double robZCenter = camPosZ - centerBoxX * sin(camTheta) + centerBoxZ * cos(camTheta);
	cand.robZMin = robZCenter - lengthBoxZ / 2;
	cand.robZMax = robZCenter + lengthBoxZ / 2;

	// camera polar coordinates
	cand.camLatMin = 0;
	cand.camLatMax = 0;
	cand.camLongMin = 0;
	cand.camLongMax = 0;

	outCand = cand;
	
	return errorCode;
}

bool candIsValid(const Candidate & inCand){	
	// left end
	if(inCand.robXMin < -0.15) return false;
	// right end
	if(inCand.robXMax > +0.15) return false;
	// bottom end
	if(inCand.robYMin < -0.05) return false;
	// top end
	if(inCand.robYMax > +0.15) return false;
	
	return true;
}

int candPrint(const Candidate & inCand){
	int errorCode = 0;

	cout << "Candidate" << endl;

	cout << " xMin: " << inCand.robXMin;
	cout << " xMax: " << inCand.robXMax;
	cout << endl;

	cout << " yMin: " << inCand.robYMin;
	cout << " yMax: " << inCand.robYMax;
	cout << endl;

	cout << " zMin: " << inCand.robZMin;
	cout << " zMax: " << inCand.robZMax;
	cout << endl;

	return errorCode;
}

int candShow(
	const vector<Candidate> & inCandVect,
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
		errorCode = candToRect(cand, outImage, rect);
		if(errorCode) return errorCode;
		
		cv::rectangle(outImage, rect, color);
	}

	return errorCode;
}

/**
* TODO
* Put this in parameter server
*/
int candToRect(
	const Candidate & inCand,
	const cv::Mat & inImage,
	cv::Rect & outRect
){
	static double camFOVLat = 45 * M_PI / 180;
	static double camFOVLong = 57.5 * M_PI / 180;

	int errorCode = 0;

	size_t imageCols = inImage.cols;
	size_t imageRows = inImage.rows;

	double minX = imageCols * (0.5 - inCand.camLongMin / camFOVLong);
	double maxX = imageCols * (0.5 - inCand.camLongMax / camFOVLong);
	double minY = imageRows * (0.5 - inCand.camLatMin / camFOVLat);
	double maxY = imageRows * (0.5 - inCand.camLatMax / camFOVLat);

	outRect = cv::Rect(
		cv::Point(minX, minY),
		cv::Point(maxX, maxY)
	);

	return errorCode;
}
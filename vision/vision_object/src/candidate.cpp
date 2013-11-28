#include <cmath>
#include <opencv2/highgui/highgui.hpp>

#include "candidate.h"

using namespace std;
using namespace vision_plane;

bool candCheckIfValid(
	const Candidate & inCand,
	const CameraContext & inContext
){
	double validLatDeg = inContext.validFovLat;
	double validLongDeg = inContext.validFovLong;

	double validLatMin = -0.5 * M_PI / 180 * validLatDeg;
	double validLatMax = +0.5 * M_PI / 180 * validLatDeg;
	double validLongMin = -0.5 * M_PI / 180 * validLongDeg;
	double validLongMax = +0.5 * M_PI / 180 * validLongDeg;

	if(inCand.minLatitude > validLatMax) return false;
	if(inCand.minLatitude < validLatMin) return false;
	if(inCand.maxLatitude > validLatMax) return false;
	if(inCand.maxLatitude < validLatMin) return false;
	if(inCand.minLongitude > validLongMax) return false;
	if(inCand.minLongitude < validLongMin) return false;
	if(inCand.maxLongitude > validLongMax) return false;
	if(inCand.maxLongitude < validLongMin) return false;
	
	return true;
}

int candShow(
	const vector<Candidate> & inCandVect,
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
		errorCode = candToRect(cand, image, rect);
		if(errorCode) return errorCode;
		
		cv::rectangle(image, rect, color);
	}
	
	cv::imshow("Candidates", image);
	cv::waitKey(0);

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

	double minX = imageCols * (0.5 - inCand.minLongitude / camFOVLong);
	double maxX = imageCols * (0.5 - inCand.maxLongitude / camFOVLong);
	double minY = imageRows * (0.5 - inCand.minLatitude / camFOVLat);
	double maxY = imageRows * (0.5 - inCand.maxLatitude / camFOVLat);

	outRect = cv::Rect(
		cv::Point(minX, minY),
		cv::Point(maxX, maxY)
	);

	return errorCode;
}

#include <cmath>
#include "candidate.h"

using namespace std;
using namespace vision_plane;

bool candCheckIfValid(
	const Candidate & inCand,
	const CameraContext & inContext
){

	double box[3][2];
	candToBox(inCand, inContext, box);
	
	// left end
	if(box[1][0] < -0.15) return false;
	// right end
	if(box[1][1] > +0.15) return false;
	// bottom end
	if(box[2][0] < -0.05) return false;
	// top end
	if(box[2][1] > +0.15) return false;
	
	return true;
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

int candToBox(
	const Candidate & inCand,
	const CameraContext & inContext,
	double outBox[3][2]
){
	int errorCode = 0;

	double minCorrLat = inCand.minLatitude + inContext.initLat;
	double maxCorrLat = inCand.maxLatitude + inContext.initLat;
	double minLong = inCand.minLongitude;
	double maxLong = inCand.maxLongitude;
	double dist = inCand.dist;

	double minX = dist * sin(minCorrLat * M_PI / 180);
	double maxX = dist * sin(maxCorrLat * M_PI / 180);
	double minY = (minX + maxX) / 2 * sin(minLong * M_PI / 180);
	double maxY = (minX + maxX) / 2 * sin(maxLong * M_PI / 180);
	double minZ = dist * cos(minCorrLat * M_PI / 180);
	double maxZ = dist * cos(minCorrLat * M_PI / 180);

	outBox[0][0] = minX;
	outBox[0][1] = maxX;
	outBox[1][0] = minY;
	outBox[1][1] = maxY;
	outBox[2][0] = minZ;
	outBox[2][1] = maxZ;

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

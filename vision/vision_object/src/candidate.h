#ifndef VISION_OBJECT_CANDIDATE
#define VISION_OBJECT_CANDIDATE

#include <vector>
#include <opencv2/core/core.hpp>
#include <vision_plane/Box.h>

#include "cameracontext.h"

class Candidate {
	public:
		double robXMin;
		double robXMax;
		double robYMin;
		double robYMax;
		double robZMin;
		double robZMax;

		double camLatMin;
		double camLatMax;
		double camLongMin;
		double camLongMax;
};

int candFilterValid(
	const std::vector<Candidate> & inCands,
	std::vector<Candidate> & outCands
);

int candFromBox(
	const vision_plane::Box & inBox,
	const CameraContext & inContext,
	Candidate & outCand
);

bool candIsValid(const Candidate & inCand);

int candPrint(const Candidate & inCand);

int candShow(
	const std::vector<Candidate> & inCandVect,
	const cv::Mat & inImage,
	cv::Mat & outImage
);

int candToRect(
	const Candidate & inCand,
	const cv::Mat & inImage,
	cv::Rect & outRect
);

#endif
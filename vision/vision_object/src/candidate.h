#ifndef VISION_OBJECT_CANDIDATE
#define VISION_OBJECT_CANDIDATE

#include <vector>
#include <opencv2/core/core.hpp>
#include <vision_plane/Candidate.h>
#include "cameracontext.h"

bool candCheckIfValid(
	const vision_plane::Candidate & inCand,
	const CameraContext & inContext
);

/*
int candFilterValid(
	const CameraContext & inContext,
	std::vector<vision_plane::Candidate> & ioCandVect
);
*/

int candShow(
	const std::vector<vision_plane::Candidate> & inCandVect,
	const cv::Mat & inImage,
	cv::Mat & outImage
);

int candToBox(
	const vision_plane::Candidate & inCand,
	const CameraContext & inContext,
	double outBox[3][2]
);

int candToRect(
	const vision_plane::Candidate & inCand,
	const cv::Mat & inImage,
	cv::Rect & outRect
);

#endif


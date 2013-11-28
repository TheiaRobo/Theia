#ifndef VISION_OBJECT_CANDIDATE
#define VISION_OBJECT_CANDIDATE

#include <vector>
#include <opencv2/core/core.hpp>
#include <vision_plane/Candidate.h>

#include "camera.h"

bool candCheckIfValid(
	const std::vector<vision_plane::Candidate> & inCandVect,
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

int candToRect(
	const vision_plane::Candidate & inCand,
	const cv::Mat & inImage,
	cv::Rect & outRect
);

#endif
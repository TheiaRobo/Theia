#ifndef VISION_OBJECT_CANDIDATE
#define VISION_OBJECT_CANDIDATE

#include <vector>
#include <opencv2/core/core.hpp>
#include <vision_plane/Candidate.h>

int candShow(
	const std::vector<vision_plane::Candidate> & inCandVect,
	const cv::Mat & inImage
);

int candToRect(
	const vision_plane::Candidate & inCand,
	const cv::Mat & inImage,
	cv::Rect & outRect
);

#endif
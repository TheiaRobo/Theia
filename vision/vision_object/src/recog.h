#ifndef VISION_OBJECT_RECOG
#define VISION_OBJECT_RECOG

#include <vector>
#include <opencv2/core/core.hpp>

#include "train.h"

int recog(
	cv::Mat & image,
	std::vector<ObjectTrainData_t> & trainDataVect
);

#endif
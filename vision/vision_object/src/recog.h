#ifndef VISION_OBJECT_RECOG
#define VISION_OBJECT_RECOG

#include <vector>
#include <vision/image.h>
#include <opencv2/core/core.hpp>

#include "train.h"

int recog(
	TheiaImageData & data,
	std::vector<ObjectTrainData_t> & trainDataVect,
	TheiaImageContext & context
);

#endif
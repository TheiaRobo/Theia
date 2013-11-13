#ifndef VISION_OBJECT_RECOG
#define VISION_OBJECT_RECOG

#include <vector>
#include <vision/array.h>
#include <vision/image.h>
#include <opencv2/core/core.hpp>

#include "train.h"

int recog(
	TheiaImageData & data,
	Array<ObjectTrainData_t> & trainDataArr,
	TheiaImageContext & context
);

#endif
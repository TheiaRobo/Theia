#ifndef VISION_OBJECT_TRAIN
#define VISION_OBJECT_TRAIN

#include <string>
#include <vision/image.h>
#include <opencv2/core/core.hpp>

#include "file.h"

typedef struct {
	std::string path;
	TheiaImageContext imageContext;
} ObjectTrainConfig_t;

typedef struct {
	ObjectFileTrain_t file;
	TheiaImageData data;
} ObjectTrainData_t;

int train(
	ObjectTrainConfig_t & trainConfig,
	std::vector<ObjectTrainData_t> & trainDataVect
);

#endif
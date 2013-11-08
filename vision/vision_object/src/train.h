#ifndef VISION_OBJECT_TRAIN
#define VISION_OBJECT_TRAIN

#include <string>
#include <opencv2/core/core.hpp>

#include "file.h"

typedef struct {
	std::string path;
	int surfMinHessian;
} ObjectTrainConfig_t;

typedef struct {
	ObjectFileTrain_t file;
	cv::Mat image;
	cv::Mat descriptors;
} ObjectTrainData_t;

int train(
	ObjectTrainConfig_t & trainConfig,
	std::vector<ObjectTrainData_t> & trainDataVect
);

#endif
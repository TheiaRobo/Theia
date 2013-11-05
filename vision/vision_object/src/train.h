#ifndef VISION_OBJECT_TRAIN
#define VISION_OBJECT_TRAIN

#include <string>

typedef struct {
	std::string path;
	int surfMinHessian;
} ObjectTrainConfig_t;

int train(ObjectTrainConfig_t & trainConfig);

#endif
#ifndef VISION_OBJECT_TRAIN
#define VISION_OBJECT_TRAIN

#include <string>

typedef struct {
	std::string path;
	int surfMinHessian;
} TrainConfig_t;

int train(TrainConfig_t & trainConfig);

#endif
#ifndef VISION_OBJECT_TRAIN
#define VISION_OBJECT_TRAIN

#include <string>

typedef struct {
	std::string path;
	std::string object;
	int rotation;
} TrainFile_t;

int train(std::string dirName);

#endif
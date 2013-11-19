#ifndef VISION_OBJECT_RECOG
#define VISION_OBJECT_RECOG

#include <vector>
#include <vision/image.h>

#include "file.h"
#include "train.h"

typedef struct {
	double minScore;
	TheiaImageContext * imageContextPtr;
} ObjectRecogContext;

int recog(
	TheiaImageData & data,
	std::vector<ObjectTrainData_t> & trainDataVect,
	ObjectRecogContext & context,
	ObjectFileTrain_t ** recognizedPtrPtr
);

#endif
#ifndef VISION_OBJECT_RECOG
#define VISION_OBJECT_RECOG

#include <vector>
#include <vision/array.h>
#include <vision/image.h>

#include "file.h"
#include "train.h"

typedef struct {
	double minScore;
	TheiaImageContext * imageContextPtr;
} ObjectRecogContext;

int recog(
	TheiaImageData & data,
	Array<ObjectTrainData_t> & trainDataArr,
	ObjectRecogContext & context,
	ObjectFileTrain_t ** recognized
);

#endif
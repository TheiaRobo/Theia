#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <vision/image.h>

int theiaImageDetectKeypoints(
	TheiaImage & image,
	TheiaImageContext & context
){
	context.detector.detect(
		image.image,
		image.keypoints
	);

	return 0;
}

int theiaImageShowKeypoints(TheiaImage & image){
	return 0;
}

int theiaImageExtractDescriptors(
	TheiaImage & image,
	TheiaImageContext & context
){
	context.extractor.compute(
		image.image,
		image.keypoints,
		image.descriptors
	);
	return 0;
}
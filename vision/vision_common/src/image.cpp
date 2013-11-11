#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <vision/image.h>

int theiaImageDetectKeypoints(
	TheiaImageData & data,
	TheiaImageContext & context
){
	context.detector.detect(
		data.image,
		data.keypoints
	);

	return 0;
}

int theiaImageShowKeypoints(TheiaImageData & data){
	return 0;
}

int theiaImageExtractDescriptors(
	TheiaImageData & data,
	TheiaImageContext & context
){
	context.extractor.compute(
		data.image,
		data.keypoints,
		data.descriptors
	);
	
	return 0;
}
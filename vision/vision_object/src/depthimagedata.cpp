#include <limits>

#include "depthimagedata.h"

using namespace cv;

DepthImageContext::DepthImageContext(const DepthImageConfig & config){
	contourMode = config.contourMode;
	contourMethod = config.contourMethod;
}

DepthImageResult DepthImageResult::worst(){
	DepthImageResult result;
	result.error = std::numeric_limits<double>::infinity();

	return result;
}

bool DepthImageResult::isBetterThan(const DepthImageResult & result){
	return (error < result.error);
}

int DepthImageData::match(
	const DepthImageData & inSample,
	const DepthImageContext & inContext,
	DepthImageResult & outResult
){
	return 0;
}

int DepthImageData::show(){
	return 0;
}

int DepthImageData::train(const DepthImageContext & inContext){
	return 0;
}

int DepthImageData::train(
	const Mat & inImage,
	const DepthImageContext & inContext
){
	return 0;
}
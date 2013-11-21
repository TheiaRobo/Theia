#include <iostream>
#include <limits>
#include <opencv2/highgui/highgui.hpp>

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
	int errorCode = 0;

	if(path.empty()){
		std::cout << "Error in " << __FUNCTION__ << std::endl;
		std::cout << "No path given" << std::endl;
		return -1;
	}

	Mat image = imread(path, CV_LOAD_IMAGE_GRAYSCALE);
	if(!image.data){
		std::cout << "Error in " << __FUNCTION__ << std::endl;
		std::cout << "Could not read image" << std::endl;
		std::cout << path << std::endl;
		return -1;
	}

	errorCode = train(image, inContext);
	if(errorCode) return errorCode;

	return errorCode;
}

int DepthImageData::train(
	const Mat & inImage,
	const DepthImageContext & inContext
){
	return 0;
}
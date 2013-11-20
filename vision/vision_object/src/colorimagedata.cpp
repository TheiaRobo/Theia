#include <iostream>
#include <opencv2/highgui/highgui.hpp>

#include "colorimagedata.h"

using namespace cv;

ColorImageContext::ColorImageContext(const ColorImageConfig & config){
	detector = SurfFeatureDetector(config.minHessian);
	extractor = SurfDescriptorExtractor();
}

int ColorImageData::train(const ColorImageContext & context){
	int errorCode = 0;

	if(path.empty()){
		std::cout << "Error in ColorImageData::train" << std::endl;
		std::cout << "No path given" << std::endl;
		return -1;
	}

	Mat image = imread(path, CV_LOAD_IMAGE_GRAYSCALE);
	if(!image.data){
		std::cout << "Error in ColorImageData::train" << std::endl;
		std::cout << "Could not read image" << std::endl;
		std::cout << path << std::endl;
		return -1;
	}

	errorCode = train(image, context);
	if(errorCode) return errorCode;

	return errorCode;
}

int ColorImageData::train(
	const Mat & inImage,
	const ColorImageContext & context
){
	int errorCode = 0;

	image = inImage;
	context.detector.detect(image, keypoints);
	context.extractor.compute(image, keypoints, descriptors);

	return errorCode;
}
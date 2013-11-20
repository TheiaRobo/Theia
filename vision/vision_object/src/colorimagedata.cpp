#include <iostream>
#include <limits>
#include <opencv2/highgui/highgui.hpp>

#include "colorimagedata.h"

using namespace cv;

ColorImageContext::ColorImageContext(const ColorImageConfig & config) :
detector(config.minHessian), extractor(), matcher(NORM_L2)
{
	// nothing
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

int ColorImageData::match(
	const ColorImageData & inSample,
	const ColorImageContext & inContext,
	ColorImageResult & outResult
){
	int errorCode = 0;

	std::vector<DMatch> matches;
  	inContext.matcher.match(descriptors, inSample.descriptors, matches);

  	size_t numbMatches;
  	numbMatches = matches.size();

  	double totalError = 0;
  	double totalSquareError = 0;
  	for(size_t i = 0; i < numbMatches; i++){
  		double distance = matches[i].distance;

  		totalError += distance;
  		totalSquareError += distance * distance;
  	}

  	double meanError;
  	double meanSquareError;
  	double variance;

  	if(numbMatches){
  		meanError = totalError / numbMatches;
  		meanSquareError = totalSquareError / numbMatches;
  		variance = meanSquareError - meanError * meanError;
  	}else{
  		meanError = std::numeric_limits<double>::infinity();
  		meanSquareError = std::numeric_limits<double>::infinity();
  		variance = 0;
  	}

  	outResult.meanError = meanError;
  	outResult.meanSquareError = meanSquareError;
  	outResult.variance = variance;
  	outResult.matches = matches;

	return errorCode;
}
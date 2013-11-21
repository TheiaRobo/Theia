#include <iostream>
#include <limits>
#include <opencv2/highgui/highgui.hpp>

#include "colorimagedata.h"

using namespace cv;

ColorImageContext::ColorImageContext(const ColorImageConfig & config) :
detector(config.minHessian), extractor(), matcher(NORM_L2) {
	// nothing
}

ColorImageResult ColorImageResult::worst(){
	ColorImageResult result;
	result.meanError = std::numeric_limits<double>::infinity();
	result.meanSquareError = std::numeric_limits<double>::infinity();
	result.variance = 0;

	return result;
}

bool ColorImageResult::isBetterThan(const ColorImageResult & result){
	return (meanSquareError < result.meanSquareError);
}

int ColorImageData::match(
	const ColorImageData & inSample,
	const ColorImageContext & inContext,
	ColorImageResult & outResult
){
	int errorCode = 0;

	outResult = ColorImageResult::worst();

	errorCode = matchKeypoints(inSample, inContext, outResult);
	if(errorCode) return errorCode;

	errorCode = matchColors(inSample, inContext, outResult);
	if(errorCode) return errorCode;

	return errorCode;
}

int ColorImageData::matchKeypoints(
	const ColorImageData & inSample,
	const ColorImageContext & inContext,
	ColorImageResult & outResult
){
	int errorCode = 0;

	std::vector<DMatch> matches;
  	inContext.matcher.match(
  		descriptors,
  		inSample.descriptors,
  		matches
  	);

  	size_t numbMatches;
  	numbMatches = matches.size();
  	if(!numbMatches){
  		return errorCode;
  	}

  	double totalError = 0;
  	double totalSquareError = 0;
  	for(size_t i = 0; i < numbMatches; i++){
  		double distance = matches[i].distance;

  		totalError += distance;
  		totalSquareError += distance * distance;
  	}

  	double meanError = totalError / numbMatches;
  	double meanSquareError = totalSquareError / numbMatches;
  	double variance = meanSquareError - meanError * meanError;

  	outResult.meanError = meanError;
  	outResult.meanSquareError = meanSquareError;
  	outResult.variance = variance;
  	outResult.matches = matches;

	return errorCode;
}

int ColorImageData::matchColors(
	const ColorImageData & inSample,
	const ColorImageContext & inContext,
	ColorImageResult & inOutResult
){
	int errorCode = 0;

	return errorCode;
}

int ColorImageData::show(){
	int errorCode = 0;

	Mat imageWithKeypoints;
	drawKeypoints(
		image,
		keypoints,
		imageWithKeypoints,
		Scalar::all(-1),
		DrawMatchesFlags::DRAW_RICH_KEYPOINTS
	);

	imshow("Keypoints", imageWithKeypoints);
	waitKey(0);

	return errorCode;
}

int ColorImageData::train(const ColorImageContext & context){
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

#include <algorithm>
#include <iostream>
#include <limits>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "colorimagedata.h"

using namespace cv;

ColorImageContext::ColorImageContext(const ColorImageConfig & config) :
detector(config.minHessian), extractor(), matcher(NORM_L2) {
	// nothing
}

ColorImageResult::ColorImageResult(){
	meanError = std::numeric_limits<double>::infinity();
	meanSquareError = std::numeric_limits<double>::infinity();
	variance = 0;
}

int ColorImageResult::getBestMatches(
	int inNumbMatches,
	std::vector<cv::DMatch> & outMatches
){
	int errorCode = 0;

	std::vector<DMatch> workingCopy(matches);
	sort(workingCopy.begin(), workingCopy.end());

	size_t totalMatches = workingCopy.size();
	size_t numbMatches = inNumbMatches;
	if(numbMatches > totalMatches){
		numbMatches = totalMatches;
	}

	outMatches = std::vector<DMatch>(
		workingCopy.begin(),
		workingCopy.begin() + numbMatches
	);

	return errorCode;
}

bool ColorImageResult::isBetterThan(const ColorImageResult & result){
	return (meanSquareError < result.meanSquareError);
}

int ColorImageData::findHomography(
	const ColorImageData & inSample,
	const ColorImageContext & inContext,
	ColorImageResult & ioResult
){
	int errorCode = 0;

  	// size_t numbGoodMatches = inContext.numbMatchesHomography;
  	size_t numbGoodMatches = 6;
  	size_t numbMatches = ioResult.matches.size();
  	if(numbMatches < numbGoodMatches){
  		std::cout << "Error in " << __FUNCTION__ << std::endl;
  		std::cout << "Not enough matches" << std::endl;
  		std::cout << path << std::endl;
  		return -1;
  	}

  	std::vector<DMatch> goodMatches;
  	errorCode = ioResult.getBestMatches(numbGoodMatches, goodMatches);
  	if(errorCode) return errorCode;

  	// construct point vectors
  	std::vector<Point2f> samplePoints;
  	std::vector<Point2f> trainPoints;
  	for(size_t i = 0; i < numbGoodMatches; i++){
  		size_t sampleIndex = goodMatches[i].trainIdx;
  		size_t trainIndex = goodMatches[i].queryIdx;

  		samplePoints.push_back(inSample.keypoints[sampleIndex].pt);
  		trainPoints.push_back(keypoints[trainIndex].pt);
  	}

  	Mat homography;
  	try{
  		homography = cv::findHomography(trainPoints, samplePoints);
  		// homography = findHomography(trainPoints, sampleKeypoints, CV_RANSAC);
  	}catch(Exception ex){
  		std::cout << "Error in " << __FUNCTION__ << std::endl;
  		std::cout << "Could not find homography" << std::endl;
  		return -1;
  	}

  	ioResult.homography = homography;

	return errorCode;
}

int ColorImageData::match(
	const ColorImageData & inSample,
	const ColorImageContext & inContext,
	ColorImageResult & outResult
){
	int errorCode = 0;

	errorCode = matchKeypoints(inSample, inContext, outResult);
	if(errorCode) return errorCode;

	errorCode = findHomography(inSample, inContext, outResult);
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

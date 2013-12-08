#include <algorithm>
#include <iostream>
#include <limits>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "colorimagedata.h"

using namespace cv;

ColorImageResult::ColorImageResult(){
	meanError = std::numeric_limits<double>::infinity();
	meanSquareError = std::numeric_limits<double>::infinity();
	variance = 0;
}

int ColorImageResult::getBestMatches(
	int inNumbMatches,
	std::vector<cv::DMatch> & outMatches
) const {
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

bool ColorImageResult::isBetterThan(const ColorImageResult & result) const {
	return isBetterThan(result.meanSquareError);
}

bool ColorImageResult::isBetterThan(double maxMeanSquareError) const {
	return (meanSquareError < maxMeanSquareError);
}

bool ColorImageResult::isGoodEnough(const ColorImageContext & inContext) const {
	return isBetterThan(inContext.maxMeanSquareError);
}

int ColorImageData::findHomography(
	const ColorImageData & inSample,
	const ColorImageContext & inContext,
	ColorImageResult & ioResult
){
	int errorCode = 0;

  	size_t numbMatches = ioResult.matches.size();
  	if(numbMatches < 4){
  		std::cout << "Error in " << __FUNCTION__ << std::endl;
  		std::cout << "Not enough keypoints for finding homography" << std::endl;
  		return -1;
  	}

  	// construct point vectors  	
  	std::vector<Point2f> objectPoints;
  	std::vector<Point2f> samplePoints;
  	for(size_t i = 0; i < numbMatches; i++){
  		size_t objectPointIndex = ioResult.matches[i].queryIdx;
  		size_t samplePointIndex = ioResult.matches[i].trainIdx;

  		objectPoints.push_back(keypoints[objectPointIndex].pt);
  		samplePoints.push_back(inSample.keypoints[samplePointIndex].pt);
  	}

  	Mat homography;
  	try{
  		homography = cv::findHomography(
  			objectPoints,
  			samplePoints,
  			CV_LMEDS,
  			10
  		);
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

/*
	errorCode = findHomography(inSample, inContext, outResult);
	if(errorCode) return errorCode;
*/
	
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

int ColorImageData::showHomography(
	const ColorImageData & inSample,
	const ColorImageResult & inResult
){
	int errorCode = 0;

	std::vector<Point2f> corners(4);
	corners[0] = cvPoint(0, 0);
	corners[1] = cvPoint(gray.cols, 0);
	corners[2] = cvPoint(gray.cols, gray.rows);
	corners[3] = cvPoint(0, gray.rows);

	std::vector<Point2f> transformedCorners(4);
	perspectiveTransform(
		corners,
		transformedCorners,
		inResult.homography
	);

	Mat imageWithHomography(inSample.gray);
	for(size_t i = 0; i < 4; i++){
		line(
			imageWithHomography,
			transformedCorners[i],
			transformedCorners[(i + 1) % 4],
			Scalar(0, 255, 0),
			4
		);
	}

	imshow("Homography", imageWithHomography);
	waitKey();

	return errorCode;
}

int ColorImageData::showKeypoints(){
	int errorCode = 0;

	Mat imageWithKeypoints;
	drawKeypoints(
		gray,
		keypoints,
		imageWithKeypoints,
		Scalar::all(-1),
		DrawMatchesFlags::DRAW_RICH_KEYPOINTS
	);

	imshow("Keypoints", imageWithKeypoints);
	waitKey(0);

	return errorCode;
}

int ColorImageData::showMatches(
	const ColorImageData & inSample,
	const ColorImageResult & inResult
){
	int errorCode = 0;

	std::vector<DMatch> bestMatches;
	errorCode = inResult.getBestMatches(10, bestMatches);
	if(errorCode) return errorCode;
	
	Mat imageWithMatches;
	drawMatches(
		gray, keypoints,
		inSample.gray, inSample.keypoints,
		bestMatches,
		imageWithMatches,
		Scalar::all(-1),
		Scalar::all(-1),
		vector<char>(),
		DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS
	);

	imshow("Matches", imageWithMatches);
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

	Mat image = imread(path, CV_LOAD_IMAGE_COLOR);
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

	cvtColor(inImage, color, CV_BGR2HLS);
	cvtColor(inImage, gray, CV_BGR2GRAY);

	context.detector.detect(gray, keypoints);
	context.extractor.compute(gray, keypoints, descriptors);

	return errorCode;
}

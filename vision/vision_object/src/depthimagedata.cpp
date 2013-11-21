#include <iostream>
#include <limits>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "depthimagedata.h"

using namespace cv;

DepthImageContext::DepthImageContext(const DepthImageConfig & config){
	blurring = config.blurring;
	contourMode = config.contourMode;
	contourMethod = config.contourMethod;
	cannyLevelOne = config.cannyLevelOne;
	cannyLevelTwo = config.cannyLevelTwo;
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
	int errorCode = 0;

	Mat imageWithContour = image.clone();
	drawContours(
		imageWithContour,
		contours,
		-1, // draw all
		Scalar(128, 255, 255)
	);

	imshow("Contours", imageWithContour);
	waitKey(0);

	return errorCode;
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
	int errorCode = 0;

	image = inImage;

	// blurring
	Mat workingImage = image.clone();
	blur(
		workingImage,
		workingImage,
		Size(inContext.blurring, inContext.blurring)
	);
	
	// edge detection
	Canny(
		workingImage,
		workingImage,
		inContext.cannyLevelOne,
		inContext.cannyLevelTwo
	);

	imshow("Canny", workingImage);
	waitKey(0);

	// contour recognition
	try{
		findContours(
			workingImage,
			contours,
			inContext.contourMode,
			inContext.contourMethod
		);
	}catch(Exception ex){
		std::cout << "Error in " << __FUNCTION__ << std::endl;
		std::cout << "Could not find contours" << std::endl;
		std::cout << path << std::endl;
		return -1;
	}

	return errorCode;
}
#include <iostream>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/features2d.hpp>

#include "recog.h"

using namespace cv;

int recogObject(
	Mat & descriptors,
	ObjectTrainData_t & trainData
);

int recogObject(
	Mat & descriptors,
	ObjectTrainData_t & trainData
){
	static FlannBasedMatcher matcher;

	/**
	* Match descriptors
	*/
  	std::vector<DMatch> matches;
  	matcher.match(
  		descriptors,
  		trainData.descriptors,
  		matches
  	);

  	size_t numbMatches;
  	numbMatches = matches.size();

  	if(!numbMatches){
  		std::cout << "Error: No matching descriptors" << std::endl;
  	}

  	/**
  	* Generate score
  	*/
  	double totalError;
  	totalError = 0;

  	double totalSquareError;
  	totalSquareError = 0;

  	size_t numbSquareErrors;
  	numbSquareErrors = matches.size();
  	
  	size_t i;
  	for(i = 0; i < numbSquareErrors; i++){
  		double distance;
  		distance = matches[i].distance;

  		totalError += distance;
  		totalSquareError += distance * distance;
  	}

  	double meanError;
  	meanError = totalError / numbSquareErrors;

  	double meanSquareError;
  	meanSquareError = totalSquareError / numbSquareErrors;

  	double variance;
  	variance = meanSquareError - meanError * meanError;

  	std::cout << "Statistics for ";
  	std::cout << trainData.file.object << std::endl;
  	std::cout << " Mean: " << meanError << std::endl;
  	std::cout << " Variance " << variance << std::endl;
  	std::cout << std::endl;

  	return 0;
}

int recog(
	Mat & image,
	std::vector<ObjectTrainData_t> & trainDataVect
){
	static SurfFeatureDetector detector(800);
	static SurfDescriptorExtractor extractor;

	std::cout << "RECOGNITION" << std::endl;
	std::cout << " Start" << std::endl;

	/**
	* Detect features
	*/
	std::vector<KeyPoint> keypoints;
	detector.detect(image, keypoints);

	/**
	* Extract descriptors
	*/
	Mat descriptors;
	extractor.compute(image, keypoints, descriptors);

	/**
	* Match with training data
	*/
	size_t numbTrainObjects;
	numbTrainObjects = trainDataVect.size();

	size_t i;
	for(i = 0; i < numbTrainObjects; i++){
		recogObject(image, trainDataVect[i]);
	}

	std::cout <<" End" << std::endl;

	return 0;
}
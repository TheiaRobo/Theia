#include <algorithm>
#include <iostream>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/features2d.hpp>

#include "recog.h"
#include "train.h"

using namespace cv;

typedef struct {
	double meanError;
	double meanSquareError;
	double variance;
} ObjectRecogScore_t;

typedef struct {
	ObjectTrainData_t * trainDataPtr;
	ObjectRecogScore_t recogScore;
} ObjectDataScorePair_t;

bool ObjectDataScorePairPtrCompare(
	ObjectDataScorePair_t * one,
	ObjectDataScorePair_t * two
);

int recogObject(
	Mat & descriptors,
	ObjectTrainData_t & trainData,
	ObjectRecogScore_t & recogScore
);

/**
* True if one is bettern than two
*/
bool ObjectDataScorePairPtrCompare(
	ObjectDataScorePair_t * one,
	ObjectDataScorePair_t * two
){
	double errors[2];
	errors[0] = one->recogScore.meanSquareError;
	errors[1] = two->recogScore.meanSquareError;

	if(errors[0] < errors[1]){
		return true;
	}else{
		return false;
	}
}

int recogObject(
	TheiaImageData & data,
	ObjectTrainData_t & trainData,
	ObjectRecogScore_t & recogScore
){
	static BFMatcher matcher(NORM_L2);

	/**
	* Match descriptors
	*/
	std::vector<DMatch> matches;
  	matcher.match(
  		trainData.data.descriptors,
  		data.descriptors,
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

  	/**
  	* Save score
  	*/
  	recogScore.meanError = meanError;
  	recogScore.meanSquareError = meanSquareError;
  	recogScore.variance = variance;

  	/**
  	* Show statistics
  	*/
  	std::cout << "Statistics for ";
  	std::cout << trainData.file.object << std::endl;
  	std::cout << " Mean error: " << meanError << std::endl;
  	std::cout << " Mean square error: " << meanSquareError << std::endl;
  	std::cout << " Variance " << variance << std::endl;
  	std::cout << std::endl;

  	return 0;
}

int recog(
	TheiaImageData & data,
	std::vector<ObjectTrainData_t> & trainDataVect,
	TheiaImageContext & context
){

	std::cout << "RECOGNITION" << std::endl;
	std::cout << " Start" << std::endl;

	std::cout << " Detect keypoints" << std::endl;
	theiaImageDetectKeypoints(data, context);

	size_t numbKeypoints;
	numbKeypoints = data.keypoints.size();
	
	if(!numbKeypoints){
		std::cout << " No keypoints found" << std::endl;
		return -1;
	}

	std::cout << " Extract descriptors" << std::endl;
	theiaImageExtractDescriptors(data, context);

	/**
	* Match with training data
	*/
	size_t numbTrainObjects;
	numbTrainObjects = trainDataVect.size();

	size_t i;
	ObjectDataScorePair_t dataScorePairArr[numbTrainObjects];

	for(i = 0; i < numbTrainObjects; i++){
		recogObject(
			data,
			trainDataVect[i],
			dataScorePairArr[i].recogScore
		);

		dataScorePairArr[i].trainDataPtr = &trainDataVect[i];

	}

	std::sort(
		dataScorePairPtrArr,
		dataScorePairPtrArr + numbTrainObjects,
		ObjectDataScorePairPtrCompare
	);


	/**
	* Show results
	*/
	std::cout << "RESULTS" << std::endl;
	for(i = 0; i < numbTrainObjects; i++){
		ObjectDataScorePair_t * pairPtr;
		pairPtr = dataScorePairPtrArr[i];

		std::cout << " Object: ";
		std::cout << pairPtr->trainDataPtr->file.object;
		std::cout << std::endl;

		std::cout << " Mean square error: ";
		std::cout << pairPtr->recogScore.meanSquareError;
		std::cout << std::endl;
	}

	std::cout << " End" << std::endl;

	return 0;
}
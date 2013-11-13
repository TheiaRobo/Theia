#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <vector>
#include <vision/array.h>

#include "recog.h"
#include "train.h"

using namespace cv;

class ObjectRecogScore {
	public:
		double meanError;
		double meanSquareError;
		double variance;

		bool operator<(double scalar) const {
			if(meanSquareError < scalar){
				return true;
			}else{
				return false;
			}
		}

		bool operator<(const ObjectRecogScore & that) const {
			if(meanSquareError < that.meanSquareError){
				return true;
			}else{
				return false;
			}
		}
};

class ObjectDataScorePair {
	public:
		ObjectTrainData_t * trainDataPtr;
		ObjectRecogScore * recogScorePtr;

		bool operator<(double scalar) const {
			ObjectRecogScore & scoreThis = *recogScorePtr;

			if(scoreThis < scalar){
				return true;
			}else{
				return false;
			}
		}

		bool operator<(const ObjectDataScorePair & that) const {
			ObjectRecogScore & scoreThis = *recogScorePtr;
			ObjectRecogScore & scoreThat = *that.recogScorePtr;

			if(scoreThis < scoreThat){
				return true;
			}else{
				return false;
			}
		}
};

/**
* Compare sample data to training data
* and calculate a score.
*/
int recogObject(
	TheiaImageData & data,
	ObjectTrainData_t & trainData,
	ObjectRecogScore & recogScore
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

  	return 0;
}

int recog(
	TheiaImageData & data,
	Array<ObjectTrainData_t> & trainDataArr,
	ObjectRecogContext & context
){
	std::cout << "RECOGNITION" << std::endl;
	std::cout << " Start" << std::endl;

	// detect keypoints
	std::cout << " Detect keypoints" << std::endl;

	TheiaImageContext & imageContext = *context.imageContextPtr;
	theiaImageDetectKeypoints(data, imageContext);

	size_t numbKeypoints;
	numbKeypoints = data.keypoints.size();
	
	if(!numbKeypoints){
		std::cout << " No keypoints found" << std::endl;
		return -1;
	}

	// extract descriptors
	std::cout << " Extract descriptors" << std::endl;
	theiaImageExtractDescriptors(data, imageContext);

	size_t numbTrainObjects;
	numbTrainObjects = trainDataArr.size();

	// calculate score
	std::cout << " Calculate score" << std::endl;
	Array<ObjectRecogScore> recogScoreArr(numbTrainObjects);

	for(size_t i = 0; i < numbTrainObjects; i++){
		recogObject(data, trainDataArr[i], recogScoreArr[i]);
	}

	// generate ranklist
	std::cout << " Generate ranklist" << std::endl;
	Array<ObjectDataScorePair> dataScorePairArr(numbTrainObjects);

	for(size_t i = 0; i < numbTrainObjects; i++){
		dataScorePairArr[i].trainDataPtr = &trainDataArr[i];
		dataScorePairArr[i].recogScorePtr = &recogScoreArr[i];
	}

	dataScorePairArr.sort();

	// filter ranklist
	std::cout << " Filter ranklist" << std::endl;
	std::vector<ObjectDataScorePair> filteredDataScorePairVect;

	for(size_t i = 0; i < numbTrainObjects; i++){
		ObjectDataScorePair & dataScorePair = dataScorePairArr[i];

		if(dataScorePair < context.minScore){
			filteredDataScorePairVect.push_back(dataScorePair);
		}
	}

	size_t numbFilteredTrainObjects;
	numbFilteredTrainObjects = filteredDataScorePairVect.size();

	// show results
	std::cout << " RESULTS" << std::endl;
	for(size_t i = 0; i < numbFilteredTrainObjects; i++){
		ObjectDataScorePair & dataScorePair = filteredDataScorePairVect[i];
		ObjectTrainData_t & trainData = *dataScorePair.trainDataPtr;
		ObjectRecogScore & recogScore = *dataScorePair.recogScorePtr;

		std::cout << "  Object: ";
		std::cout << trainData.file.object;
		std::cout << std::endl;

		std::cout << "  Mean square error: ";
		std::cout << recogScore.meanSquareError;
		std::cout << std::endl;
	}

	std::cout << " End" << std::endl;

	return 0;
}
#include <iostream>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>

#include "file.h"
#include "recog.h"
#include "train.h"

using namespace cv;

class ObjectRecogScore {
	public:
		double meanError;
		double meanSquareError;
		double variance;
		std::vector<DMatch> matches;

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
  	recogScore.matches = matches;

  	return 0;
}

int recogFindHomography(
	TheiaImageData & sampleData,
	ObjectTrainData_t & trainData,
	ObjectRecogScore & recogScore,
	Mat & homography
){
	std::cout << "FIND HOMOGRAPHY" << std::endl;

  	std::vector<KeyPoint> & sampleKeypoints = sampleData.keypoints;
  	std::vector<KeyPoint> & trainKeypoints = trainData.data.keypoints;

  	std::vector<DMatch> & matches = recogScore.matches;

  	size_t numbMatches;
  	numbMatches = matches.size();

  	double minDist = 1e6;
  	double maxDist = 0;
  	for(size_t i = 0; i < numbMatches; i++){
  		double dist = matches[i].distance;

  		if(dist < minDist){
  			minDist = dist;
  		}

  		if(dist > maxDist){
  			maxDist = dist;
  		}
  	}

  	std::vector<DMatch> goodMatches;
  	for(size_t i = 0; i < numbMatches; i++){
  		if(matches[i].distance < 2.5 * minDist){
  			goodMatches.push_back(matches[i]);
  		}
  	}

  	size_t numbGoodMatches;
  	numbGoodMatches = goodMatches.size();

	std::cout << " Numb matches: " << numbMatches << std::endl;
  	std::cout << " Numb good matches: " << numbGoodMatches << std::endl;

  	// construct point vectors
  	std::vector<Point2f> trainPoints;
  	std::vector<Point2f> samplePoints;
  	for(size_t i = 0; i < numbGoodMatches; i++){
  		trainPoints.push_back(trainKeypoints[goodMatches[i].queryIdx].pt);
  		samplePoints.push_back(sampleKeypoints[goodMatches[i].trainIdx].pt);
  	}

  	std::cout << " Numb train points: " << trainPoints.size() << std::endl;
  	std::cout << " Numb sample points: " << samplePoints.size() << std::endl;

  	homography = findHomography(trainPoints, samplePoints);
  	// homography = findHomography(trainPoints, sampleKeypoints, CV_RANSAC);

  	std::cout << "End" << std::endl;

  	return 0;
}

int recogShowHomography(
	TheiaImageData & data,
	ObjectTrainData_t & trainData,
	Mat & homography
){
	Mat & cameraImage = data.image;
	Mat & objectImage = trainData.data.image;
	
	std::cout << "SHOW HOMOGRAPHY" << std::endl;
	
	// construct corner vector
	std::cout << " Construct corner vector" << std::endl;

	std::vector<Point2f> corners(4);
	corners[0] = cvPoint(0, 0);
	corners[1] = cvPoint(objectImage.cols, 0);
	corners[2] = cvPoint(objectImage.cols, objectImage.rows);
	corners[3] = cvPoint(0, objectImage.rows);

	// transform corners
	std::cout << " Transform corners" << std::endl;

	std::vector<Point2f> transformedCorners(4);
	perspectiveTransform(
		corners,
		transformedCorners,
		homography
	);

	// draw lines
	Mat image(cameraImage);
	for(size_t i = 0; i < 4; i++){
		line(
			image,
			transformedCorners[i],
			transformedCorners[(i + 1) % 4],
			Scalar(0, 255, 0),
			4
		);
	}

	imshow("Object", image);
	waitKey();

	return 0;
}

int recog(
	TheiaImageData & data,
	std::vector<ObjectTrainData_t> & trainDataVect,
	ObjectRecogContext & context,
	ObjectFileTrain_t ** recognizedPtrPtr
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
	numbTrainObjects = trainDataVect.size();

	// calculate score
	std::cout << " Calculate score" << std::endl;
	ObjectRecogScore recogScoreArr[numbTrainObjects];

	for(size_t i = 0; i < numbTrainObjects; i++){
		ObjectTrainData_t & trainData = trainDataVect[i];
		ObjectRecogScore & recogScore = recogScoreArr[i];
		recogObject(data, trainData, recogScore);
	
		std::cout << " Object: " << trainData.file.object << std::endl;
		std::cout << " MSE: " << recogScore.meanSquareError << std::endl;
	}

	// generate ranklist
	std::cout << " Generate ranklist" << std::endl;
	ObjectDataScorePair dataScorePairArr[numbTrainObjects];

	for(size_t i = 0; i < numbTrainObjects; i++){
		dataScorePairArr[i].trainDataPtr = &trainDataVect[i];
		dataScorePairArr[i].recogScorePtr = &recogScoreArr[i];
	}

	std::sort(
		dataScorePairArr,
		dataScorePairArr + numbTrainObjects
	);

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

	if(!numbFilteredTrainObjects){
		*recognizedPtrPtr = NULL;
		return 0;
	}

	ObjectDataScorePair & bestScorePair = filteredDataScorePairVect[0];
	ObjectTrainData_t & bestTrainData = *bestScorePair.trainDataPtr;
	ObjectRecogScore & bestRecogScore = *bestScorePair.recogScorePtr;
	ObjectFileTrain_t & bestFile = bestTrainData.file;

	// find homography
	std::cout << " Find homography" << std::endl;
	
	Mat homography;
	recogFindHomography(
		data,
		bestTrainData,
		bestRecogScore,
		homography
	);

	// show homography
/*
	std::cout << " Show homograph" << std::endl;

	recogShowHomography(
		data,
		bestTrainData,
		homography
	);
*/

	// return result
	*recognizedPtrPtr = &bestFile;

	return 0;
}
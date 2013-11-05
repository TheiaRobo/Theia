#include <iostream>
#include <string>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/nonfree/features2d.hpp>

#include <vision/file.h>

#include "file.h"
#include "train.h"

using namespace cv;

int trainFile(
	ObjectFileTrain_t & trainFile,
	TrainConfig_t & trainConfig
);

int trainFile(
	ObjectFileTrain_t & trainFile,
	TrainConfig_t & trainConfig
){
	std::cout << "TRAINING" << std::endl;
	std::cout << " Path: " << trainFile.path << std::endl;

	/**
	* Read image
	*/
	Mat image;
	image = imread(trainFile.path, IMREAD_GRAYSCALE);

	if(!image.data){
		std::cout << "ERROR: Could not read image" << std::endl;
		return -1;
	}

	/**
	* Extract features from image
	*/
	SurfFeatureDetector detector(trainConfig.surfMinHessian);
	std::vector<KeyPoint> keypoints;

	detector.detect(image, keypoints);

	std::cout << " Number of keypoints: " << keypoints.size() << std::endl;

	/**
	* Draw keypoints
	*/
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

	/**
	* Calculate corresponding descriptors
	*/
	SurfDescriptorExtractor extractor;
	Mat descriptors;

	extractor.compute(image, keypoints, descriptors);

	return 0;
}

/**
* Find and list training files
*/
int train(TrainConfig_t & trainConfig){
	int errorCode;

	/**
	* Find files in training directory
	*/
	std::vector<std::string> fileVect;
	errorCode = visionFileScanDir(trainConfig.path, fileVect);

	if(errorCode){
		std::cout << "Error: Could not scan training directory" << std::endl;
		return errorCode;
	}

	/**
	* Filter training files
	*/
	std::vector<ObjectFileTrain_t> trainFileVect;
	objectFilesFilter(fileVect, trainFileVect);
	objectFilesShow(trainFileVect);

	/**
	* Extract features from training images
	*/
	size_t numbTrainFiles;
	numbTrainFiles = trainFileVect.size();

	size_t i;
	for(i = 0; i < numbTrainFiles; i++){
		errorCode = trainFile(trainFileVect[i], trainConfig);

		if(errorCode){
			std::cout << "Error: Could not train image" << std::endl;
			return errorCode;
		}
	}

	return 0;
}
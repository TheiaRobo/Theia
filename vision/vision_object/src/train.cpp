#include <iostream>
#include <string>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <vision/file.h>
#include <vision/image.h>

#include "file.h"
#include "train.h"

using namespace cv;

int trainFile(
	ObjectFileTrain_t & trainFile,
	ObjectTrainData_t & trainData,
	TheiaImageContext & context
);

int trainFile(
	ObjectFileTrain_t & trainFile,
	ObjectTrainData_t & trainData,
	TheiaImageContext & context
){
	std::cout << "trainFile" << std::endl;
	std::cout << " Path: " << trainFile.path << std::endl;

	/**
	* Read image
	*/
	Mat & image = trainData.data.image;
	image = imread(trainFile.path, IMREAD_GRAYSCALE);

	if(!image.data){
		std::cout << "ERROR: Could not read image" << std::endl;
		return -1;
	}

	/**
	* Extract features from image
	*/
	theiaImageDetectKeypoints(trainData.data, context);

	std::vector<KeyPoint> & keypoints = trainData.data.keypoints;
	std::cout << " # keypoints: " << keypoints.size() << std::endl;

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
	theiaImageExtractDescriptors(trainData.data, context);

	/**
	* Store data
	*/
	trainData.file = trainFile;

	return 0;
}

/**
* Find and list training files
*/
int train(
	ObjectTrainConfig_t & trainConfig,
	std::vector<ObjectTrainData_t> & trainDataVect
){
	int errorCode;

	// Remove old data
	trainDataVect.clear();

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
		ObjectTrainData_t trainData;
		errorCode = trainFile(
			trainFileVect[i],
			trainData,
			trainConfig.imageContext
		);

		if(errorCode){
			std::cout << "Error: Could not train image" << std::endl;
			return errorCode;
		}

		trainDataVect.push_back(trainData);
	}

	return 0;
}
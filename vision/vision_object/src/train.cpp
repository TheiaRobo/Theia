#include <iostream>
#include <string>
#include <vector>

#include <vision/file.h>

#include "file.h"
#include "train.h"

using namespace std;


/**
* Find and list training files
*/
int train(TrainConfig_t & trainConfig){
	int errorCode;

	/**
	* Find files in training directory
	*/
	vector<string> fileVect;
	errorCode = visionFileScanDir(trainConfig.path, fileVect);

	if(errorCode){
		cout << "Error: Could not scan training directory" << endl;
		return errorCode;
	}

	/**
	* Filter training files
	*/
	vector<ObjectFileTrain_t> trainFileVect;
	objectFilesFilter(fileVect, trainFileVect);
	objectFilesShow(trainFileVect);

	/**
	* Extract features from training images
	*/

	return 0;
}
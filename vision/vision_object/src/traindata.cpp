#include <cstdlib>
#include <vision/file.h>

#include "traindata.h"

using namespace std;

int TrainData::find(
	const string inPath,
	vector<TrainData> & outTrainDataVect
){
	int errorCode = 0;

	outTrainDataVect.clear();

	vector<string> dirVect;
	errorCode = visionFileFindDirs(inPath, dirVect);
	if(errorCode) return errorCode;

	size_t numbFiles = dirVect.size();
	for(size_t i = 0; i < numbFiles; i++){
		TrainData trainData;
		string dirName = dirVect[i];

		// directory name must be a number
		size_t dirNameSize = dirName.size();
		bool onlyDigits = true;
		for(size_t i = 0; i < dirNameSize; i++){
			if(!isdigit(dirName[i])){
				onlyDigits = false;
				break;
			}
		}

		if(!onlyDigits){
			continue;
		}

		trainData.angle = strtoul(dirName.c_str(), NULL, 10);
		trainData.path = inPath + VISION_DIR_SEP + dirName;

		string colorImagePath = trainData.path + VISION_DIR_SEP + "color.png";
		string depthImagePath = trainData.path + VISION_DIR_SEP + "depth.png";

		if(
			!visionFileExists(colorImagePath)
			|| !visionFileExists(depthImagePath)
		){
			continue;
		}

		trainData.colorImageData.path = colorImagePath;
		trainData.depthImageData.path = depthImagePath;

		outTrainDataVect.push_back(trainData);
	}

	return errorCode;
}

int TrainData::train(){
	int errorCode = 0;

	errorCode = colorImageData.train();
	if(errorCode) return errorCode;

	errorCode = depthImageData.train();
	if(errorCode) return errorCode;
	
	return errorCode;
}
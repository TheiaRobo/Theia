#include <cstdlib>
#include <vision/file.h>

#include "objectdata.h"

using namespace std;

Context::Context(const Config & config)
: colorImage(config.colorImage), depthImage(config.depthImage) {
	// nothing
}

ObjectDataResult ObjectDataResult::worst(){
	ObjectDataResult result;
	result.colorImage = ColorImageResult::worst();
	// result.depthImage = DepthImageResult::worst();

	return result;
}

bool ObjectDataResult::isBetterThan(const ObjectDataResult & result){
	return colorImage.isBetterThan(result.colorImage);
}

int ObjectData::find(
	const string inPath,
	vector<ObjectData> & outObjectDataVect
){
	int errorCode = 0;

	outObjectDataVect.clear();

	vector<string> dirVect;
	errorCode = visionFileFindDirs(inPath, dirVect);
	if(errorCode) return errorCode;

	size_t numbFiles = dirVect.size();
	for(size_t i = 0; i < numbFiles; i++){
		ObjectData objectData;
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

		objectData.angle = strtoul(dirName.c_str(), NULL, 10);
		objectData.path = inPath + VISION_DIR_SEP + dirName;

		string colorImagePath = objectData.path + VISION_DIR_SEP + "color.png";
		string depthImagePath = objectData.path + VISION_DIR_SEP + "depth.png";

		if(
			!visionFileExists(colorImagePath)
			|| !visionFileExists(depthImagePath)
		){
			continue;
		}

		objectData.colorImage.path = colorImagePath;
		objectData.depthImage.path = depthImagePath;

		outObjectDataVect.push_back(objectData);
	}

	return errorCode;
}

int ObjectData::train(const Context & context){
	int errorCode = 0;

	errorCode = colorImage.train(context.colorImage);
	if(errorCode) return errorCode;

	errorCode = depthImage.train(context.depthImage);
	if(errorCode) return errorCode;
	
	return errorCode;
}

int ObjectData::match(
	const ObjectData & inSample,
	const Context & inContext,
	ObjectDataResult & outResult
){
	int errorCode = 0;

	errorCode = colorImage.match(
		inSample.colorImage,
		inContext.colorImage,
		outResult.colorImage
	);
	if(errorCode) return errorCode;

	errorCode = depthImage.match(
		inSample.depthImage,
		inContext.depthImage,
		outResult.depthImage
	);
	if(errorCode) return errorCode;

	errorCode = matchColors(inSample, inContext, outResult);
	if(errorCode) return errorCode;

	return errorCode;
}

int ObjectData::matchColors(
	const ObjectData & inSample,
	const Context & inContext,
	ObjectDataResult & ioResult
){
	int errorCode = 0;
	return errorCode;
}
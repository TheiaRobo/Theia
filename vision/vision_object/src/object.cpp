#include <vision/file.h>

#include "object.h"

using namespace std;

int Object::find(
	const string inPath,
	vector<Object> & outObjectVect
){
	int errorCode = 0;

	vector<std::string> dirVect;
	errorCode = visionFileFindDirs(inPath, dirVect);
	if(errorCode) return errorCode;

	size_t numbFiles = dirVect.size();

	outObjectVect.clear();
	outObjectVect.reserve(numbFiles);

	for(size_t i = 0; i < numbFiles; i++){
		Object & object = outObjectVect[i];
		object.name = dirVect[i];
		object.path = inPath + VISION_DIR_SEP + dirVect[i];
		
		errorCode = ImageData::find(object.path, object.imageDataVect);
		if(errorCode) return errorCode;
	}

	return errorCode;	
}

int objectsTrain(vector<Object> & inOutObjectVect){
	int errorCode = 0;

	size_t numbObjects = inOutObjectVect.size();
	for(size_t i = 0; i < numbObjects; i++){
		errorCode = inOutObjectVect[i].train();
		if(errorCode) return errorCode;
	}

	return errorCode;
}

int Object::train(){
	int errorCode = 0;

	return errorCode;
}
#include <vision/file.h>

#include "object.h"

using namespace std;

int Object::find(
	const string inPath,
	vector<Object> & outObjectVect
){
	int errorCode = 0;

	outObjectVect.clear();

	vector<string> dirVect;
	errorCode = visionFileFindDirs(inPath, dirVect);
	if(errorCode) return errorCode;

	size_t numbFiles = dirVect.size();
	for(size_t i = 0; i < numbFiles; i++){
		Object object;
		object.name = dirVect[i];
		object.path = inPath + VISION_DIR_SEP + dirVect[i];

		errorCode = ObjectData::find(object.path, object.objectDataVect);
		if(errorCode) return errorCode;

		outObjectVect.push_back(object);
	}

	return errorCode;
}

int Object::train(const Context & context){
	int errorCode = 0;

	size_t numbData = objectDataVect.size();
	for(size_t i = 0; i < numbData; i++){
		errorCode = objectDataVect[i].train(context);
		if(errorCode) return errorCode;
	}
	
	return errorCode;
}

int Object::match(
	const ObjectData & inSampleData,
	const Context & inContext,
	vector<ObjectDataResult> & outResultVect
){
	int errorCode = 0;

	outResultVect.clear();

	size_t numbData = objectDataVect.size();
	for(size_t i = 0; i < numbData; i++){
		ObjectDataResult objectDataResult;

		errorCode = objectDataVect[i].match(
			inSampleData,
			inContext,
			objectDataResult
		);
		if(errorCode) return errorCode;

		outResultVect.push_back(objectDataResult);
	}

	return errorCode;
}
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

		errorCode = TrainData::find(object.path, object.trainDataVect);
		if(errorCode) return errorCode;

		outObjectVect.push_back(object);
	}

	return errorCode;
}

int Object::train(const TrainContext & context){
	int errorCode = 0;

	size_t numbImages = trainDataVect.size();
	for(size_t i = 0; i < numbImages; i++){
		errorCode = trainDataVect[i].train(context);
		if(errorCode) return errorCode;
	}
	
	return errorCode;
}
#include <iostream>
#include <string>
#include <vector>

#include "object.h"

using namespace std;

Config config;
Context context(config);
vector<Object> objectVect;

int init(){
	int errorCode = 0;

	/**
	* TODO
	* Copy configuration options from parameter server into
	* config class
	*/
	config = Config();
	config.colorImage.minHessian = 400;

	context = Context(config);

	return errorCode;
}

int match(const ObjectData & inSampleData){
	int errorCode = 0;

	size_t numbObjects = objectVect.size();
	vector<ObjectDataResult> resultVect(numbObjects);

	for(size_t i = 0; i < numbObjects; i++){
		ObjectDataResult result;

		errorCode = objectVect[i].match(inSampleData, context, result);
		if(errorCode) return errorCode;

		resultVect.push_back(result);
	}

	return errorCode;
}

int train(){
	int errorCode = 0;
	string trainPath = "/home/amotta/Documents/Theia/vision/vision_object/train";

	errorCode = Object::find(trainPath, objectVect);
	if(errorCode) return errorCode;

	size_t numbObjects = objectVect.size();
	for(size_t i = 0; i < numbObjects; i++){
		Object & object = objectVect[i];
		size_t numbData = object.objectDataVect.size();

		cout << "Object " << i << endl;
		cout << " Name: " << object.name << endl;
		cout << " # Data: " << numbData << endl;
		
		cout << " Train .." << endl;
		object.train(context);
		
		cout << " Show results .." << endl;
		for(size_t j = 0; j < numbData; j++){
			ObjectData & data = object.objectDataVect[j];
			data.colorImage.show();
		}

	}

	return errorCode;
}

int main(int argc, char ** argv){
	int errorCode = 0;

	errorCode = init();
	if(errorCode) return errorCode;

	errorCode = train();
	if(errorCode) return errorCode;

/*
	errorCode = match();
	if(errorCode) return;
*/

	return errorCode;
}
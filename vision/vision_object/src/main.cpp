#include <iostream>
#include <string>
#include <vector>

#include "object.h"

using namespace std;

int main(int argc, char ** argv){
	int errorCode = 0;
	string trainPath = "/home/amotta/Documents/Theia/vision/vision_object/train";

	vector<Object> objectVect;
	errorCode = Object::find(trainPath, objectVect);

	if(errorCode){
		cout << "Could not find objects" << endl;
		return errorCode;
	}

	size_t numbObjects = objectVect.size();
	for(size_t i = 0; i < numbObjects; i++){
		Object & object = objectVect[i];
		cout << "Object " << i << endl;
		cout << " Name: " << object.name << endl;
		cout << " # Train data: " << object.trainDataVect.size() << endl;
		
		cout << " Train .." << endl;
		object.train();
		cout << " Done!" << endl;

	}

	return errorCode;
}
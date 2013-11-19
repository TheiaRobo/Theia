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
		cout << "Object " << i << endl;
		cout << " Name: " << objectVect[i].name << endl;
		cout << " # Images: " << objectVect[i].imageDataVect.size() << endl;
	}

	return errorCode;
}
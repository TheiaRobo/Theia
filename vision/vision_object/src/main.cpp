#include <iostream>
#include <string>
#include <vector>

#include "train.h"

using namespace std;

string trainDir;

void init(){
	/*
	* TODO
	* Move to config server
	*/
	trainDir = "/home/amotta/Documents/Theia/vision/vision_object/train";
}

int main(int argc, char ** argv){
	int errorCode;

	init();

	ObjectTrainConfig_t trainConfig;

	/**
	* TODO
	* Move to config server
	*/
	trainConfig.path = trainDir;
	trainConfig.surfMinHessian = 800;

	errorCode = train(trainConfig);

	if(errorCode){
		cout << "Error: Training failed" << endl;
		return errorCode;
	}

	return 0;
}
#include <iostream>
#include <string>
#include <vector>

#include "train.h"

using namespace std;

string trainDir;

int main(int argc, char ** argv){
	int errorCode;

	/*
	* TODO
	* Move to config server
	*/
	trainDir = "/home/amotta/Documents/Theia/vision/vision_object/train";

	errorCode = train(trainDir);
	
	if(errorCode){
		cout << "Error: Training failed" << endl;
		return errorCode;
	}

	return 0;
}
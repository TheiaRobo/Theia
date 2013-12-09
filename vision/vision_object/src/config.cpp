#include <iostream>
#include <string>
#include <ros/ros.h>

#include "config.h"

using namespace std;

int configBuild(Config & outConfig){
	int errorCode = 0;

	string prefix = "~config/";
	ros::param::getCached(prefix + "path", outConfig.path);

	errorCode = cameraConfigBuild(outConfig.camera);
	if(errorCode){
		cout << "Error in " << __FUNCTION__ << endl;
		cout << "Could not build camera configuration" << endl;
		return errorCode;
	}

	errorCode = candidateConfigBuild(outConfig.candidate);
	if(errorCode){
		cout << "Error in " << __FUNCTION__ << endl;
		cout << "Could not build candidate configuration" << endl;
		return errorCode;
	}

	errorCode = colorImageConfigBuild(outConfig.colorImage);
	if(errorCode){
		cout << "Error in " << __FUNCTION__ << endl;
		cout << "Could not build color image configuration" << endl;
		return errorCode;
	}

	return errorCode;
}
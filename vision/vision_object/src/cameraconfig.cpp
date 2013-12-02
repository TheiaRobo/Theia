#include <string>
#include <ros/ros.h>

#include "cameraconfig.h"

using namespace std;

int cameraConfigBuild(CameraConfig & outConfig){
	int errorCode = 0;

	string prefix = "~config/camera";
	ros::param::getCached(prefix + "totalFOVLat", outConfig.fovLat);
	ros::param::getCached(prefix + "totalFOVLong", outConfig.fovLong);
	ros::param::getCached(prefix + "validFOVLat", outConfig.validFovLat);
	ros::param::getCached(prefix + "validFOVLong", outConfig.validFovLong);
	
	return errorCode;
}
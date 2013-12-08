#include <string>
#include <ros/ros.h>

#include "cameraconfig.h"

using namespace std;

int cameraConfigBuild(CameraConfig & outConfig){
	int errorCode = 0;

	string prefix = "~config/camera";
	ros::param::getCached(prefix + "posX", outConfig.posX);
	ros::param::getCached(prefix + "posY", outConfig.posY);
	ros::param::getCached(prefix + "posZ", outConfig.posZ);
	ros::param::getCached(prefix + "angle", outConfig.angle);
	ros::param::getCached(prefix + "totalFOVLat", outConfig.fovLat);
	ros::param::getCached(prefix + "totalFOVLong", outConfig.fovLong);
	
	return errorCode;
}
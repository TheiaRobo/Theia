#include <string>
#include <ros/ros.h>

#include "colorimageconfig.h"

using namespace std;

int colorImageConfigBuild(ColorImageConfig & outConfig){
	int errorCode = 0;

	string prefix = "~config/colorImage/"; 
	ros::param::getCached(
		prefix + "minHessian",
		outConfig.minHessian
	);
	ros::param::getCached(
		prefix + "maxMeanSquareError",
		outConfig.maxMeanSquareError
	);
	ros::param::getCached(
		prefix + "numbMatchesHomography",
		outConfig.numbMatchesHomography
	);

	return errorCode;
}
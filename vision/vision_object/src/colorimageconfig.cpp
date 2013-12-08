#include <string>
#include <ros/ros.h>

#include "colorimageconfig.h"

using namespace std;

int colorImageConfigBuild(ColorImageConfig & outConfig){
	int errorCode = 0;

	string prefix = "~config/colorImage/"; 
	ros::param::getCached(
		prefix + "histBins",
		outConfig.histBins
	);
	ros::param::getCached(
		prefix + "minHessian",
		outConfig.minHessian
	);
	ros::param::getCached(
		prefix + "maxTotalError",
		outConfig.maxTotalError
	);
	ros::param::getCached(
		prefix + "numbMatchesHomography",
		outConfig.numbMatchesHomography
	);

	return errorCode;
}
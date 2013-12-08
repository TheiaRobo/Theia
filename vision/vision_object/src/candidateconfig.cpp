#include <string>
#include <ros/ros.h>

#include "candidateconfig.h"

using namespace std;

int candidateConfigBuild(CandidateConfig & outConfig){
	int errorCode = 0;

	string prefix = "~config/cand";
	ros::param::getCached(prefix + "minX", outConfig.minX);
	ros::param::getCached(prefix + "maxX", outConfig.maxX);
	ros::param::getCached(prefix + "minY", outConfig.minY);
	ros::param::getCached(prefix + "maxY", outConfig.maxY);
	ros::param::getCached(prefix + "minZ", outConfig.minZ);
	ros::param::getCached(prefix + "maxZ", outConfig.maxZ);
	
	return errorCode;
}
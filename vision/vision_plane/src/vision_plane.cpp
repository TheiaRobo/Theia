#include <cmath>
#include <iostream>
#include <vector>
#include <pcl/common/centroid.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ros/conversions.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/PointCloud2.h>
#include <vision/cloud.h>

#include "vision_plane.h"

#define NODE_NAME "vision_plane"
#define TOPIC_IN "/camera/depth_registered/points"
#define TOPIC_OUT "/vision/plane/angle"
#define TOPIC_DEBUG_CROPPED_OUT "/vision/plane/debug/cropped"
#define TOPIC_DEBUG_NON_PLANE_OUT "/vision/plane/debug/nonPlane"
#define TOPIC_DEBUG_PLANE_OUT "/vision/plane/debug/plane"

using namespace pcl;

/**
* Type definitions
*/
typedef struct {
	double normal[3];
	double centroid[3];
	double area;
	TheiaCloudPtr cloudPtr;
} Plane;

/**
* Global variables
*/
Config config;
ros::Subscriber cloudSub;
ros::Publisher debugCroppedPub;
ros::Publisher debugNonPlanePub;
ros::Publisher debugPlanePub;
ros::Publisher anglePub;

/**
* Cropping the cloud
* The config values are taken from the parameter server
*/
void cropCloud(TheiaCloudPtr in, TheiaCloudPtr out){
	double centroid[3][2];
	ros::param::getCached("~config/crop/minX", centroid[0][0]);
	ros::param::getCached("~config/crop/maxX", centroid[0][1]);
	ros::param::getCached("~config/crop/minY", centroid[1][0]);
	ros::param::getCached("~config/crop/maxY", centroid[1][1]);
	ros::param::getCached("~config/crop/minZ", centroid[2][0]);
	ros::param::getCached("~config/crop/maxZ", centroid[2][1]);

	visionCloudCrop(in, out, centroid);
}

/**
* Scaling down the cloud
* This should improve performance for future operations
*/
void scaleCloud(TheiaCloudPtr in, TheiaCloudPtr out){
	// config
	static double leafSize = 0.02;

	pcl::VoxelGrid<TheiaPoint> grid;
	grid.setLeafSize(leafSize, leafSize, leafSize);

	grid.setInputCloud(in);
	grid.filter(*out);
}

/**
* This function tries to find all planes in a point cloud.
* A RANSAC algorithm is used for this.
*
* Heavily inspired by
* http://www.pointclouds.org/documentation/tutorials/extract_indices.php
*/
void findPlanes(TheiaCloudPtr in, std::vector<Plane> & planeVect){
	size_t origCloudSize = in->points.size();
	size_t minPlanePoints = config.minPercentage * origCloudSize;

	if(origCloudSize < 3){
		ROS_INFO("Input cloud for findPlanes is too small");
		ROS_INFO("Check cropping and rescaling parameters");
		return;
	}

	TheiaCloudPtr allPlanesCloudPtr(new TheiaCloud());
	PointIndices::Ptr inliers(new pcl::PointIndices());
	ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());

	SACSegmentation<TheiaPoint> seg;
	seg.setModelType(SACMODEL_PLANE);
	seg.setMethodType(SAC_RANSAC);
	seg.setMaxIterations(config.numbIterations);
	seg.setDistanceThreshold(config.planeDistThresh);
	seg.setOptimizeCoefficients(config.planeOptimize);

	do{

		seg.setInputCloud(in);
		seg.segment(*inliers, *coefficients);

		/**
		* Stop the search for planes when the found plane
		* is too small (in terms of points).
		*/
		size_t numbInliers = inliers->indices.size();

		if(!numbInliers) break;
		if(numbInliers < minPlanePoints) break;

		/**
		* Find normal
		*/
		Plane plane;
		plane.normal[0] = coefficients->values[0];
		plane.normal[1] = coefficients->values[1];
		plane.normal[2] = coefficients->values[2];

		/**
		* Find centroid
		*/
		Eigen::Vector4f centroid;
		pcl::compute3DCentroid(*in, *inliers, centroid);

		plane.centroid[0] = centroid[0];
		plane.centroid[1] = centroid[1];
		plane.centroid[2] = centroid[2];

		/**
		* Find area
		*/
		Eigen::Vector4f minVect;
		Eigen::Vector4f maxVect;
		pcl::getMinMax3D(*in, *inliers, minVect, maxVect);

		Eigen::Vector4f diffVect;
		for(int i = 0; i < 3; i++){
			diffVect[i] = maxVect[i] - minVect[i];
		}

		double planeHeight = diffVect[1];
		double planeBaseline = sqrt(
			diffVect[0] * diffVect[0] + diffVect[2] * diffVect[2]
		);

		plane.area = planeHeight * planeBaseline;

		/**
		* Finish plane
		*/
		planeVect.push_back(plane);

		pcl::ExtractIndices<TheiaPoint> extract;
		extract.setInputCloud(in);
		extract.setIndices(inliers);

		/**
		* Get point cloud from plane
		*/
		TheiaCloudPtr planeCloudPtr(new TheiaCloud());
		extract.setNegative(false);
		extract.filter(*planeCloudPtr);

		allPlanesCloudPtr->operator+=(*planeCloudPtr);

		/**
		* Remove plane from input cloud
		*/
		TheiaCloudPtr remainingCloudPtr(new TheiaCloud());
		extract.setNegative(true);
		extract.filter(*remainingCloudPtr);

		/**
		* Continue search 
		* But only use remaining point cloud
		*/
		in->swap(*remainingCloudPtr);
	}while(true);

	// debug
	visionCloudDebug(allPlanesCloudPtr, debugPlanePub);
	visionCloudDebug(in, debugNonPlanePub);
}

/**
* This function takes a a vector of planes
* and calculates an angle for it.
*/
double planeVectToAngle(std::vector<Plane> & planeVect){
	double totalArea = 0;
	double totalAngle = 0;
	for(unsigned int i = 0; i < planeVect.size(); i++){
		double planeNormalX = planeVect[i].normal[0];
		double planeNormalZ = planeVect[i].normal[2];
		double angle = atan2(planeNormalZ, planeNormalX);

		if(angle < 0){
			angle = angle + 2 * M_PI;
		}

		// forward transform
		angle = angle + M_PI / 4;

		int quadrant = (int) (angle / (M_PI / 2));
		angle = angle - quadrant * (M_PI / 2);

		// backwards transform
		angle = angle - M_PI / 4;

		double planeArea = planeVect[i].area;
		totalArea += planeArea;
		totalAngle += angle * planeArea;

		ROS_INFO("Plane #%u, Angle %f", i, angle);
	}

	double averageAngle = totalAngle / totalArea;
	return averageAngle;
}

void initConfig(){
	ros::param::getCached(
		"~config/numbIterations",
		config.numbIterations
	);
	ros::param::getCached(
		"~config/planeDistThresh",
		config.planeDistThresh
	);
	ros::param::getCached(
		"~config/planeOptimize",
		config.planeOptimize
	);
	ros::param::getCached(
		"~config/minPercentage",
		config.minPercentage
	);
}

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr & rosMsgPtr){
	initConfig();

	TheiaCloudPtr cloudPtr(new TheiaCloud());
	pcl::fromROSMsg(*rosMsgPtr, *cloudPtr);

	TheiaCloudPtr scaledCloudPtr(new TheiaCloud());
	scaleCloud(cloudPtr, scaledCloudPtr);
	cloudPtr.reset();

	std::vector<Plane> planeVect;
	findPlanes(scaledCloudPtr, planeVect);
	scaledCloudPtr.reset();

	if(planeVect.empty()){
		return;
	}

	ROS_INFO("- End");
}

int main (int argc, char ** argv){
	initConfig();

	// init ROS
	ros::init(argc, argv, NODE_NAME);
	ros::NodeHandle node;

	cloudSub = node.subscribe(TOPIC_IN, 1, cloudCallback);
	debugCroppedPub = node.advertise<sensor_msgs::PointCloud2>(
		TOPIC_DEBUG_CROPPED_OUT, 1
	);
	debugNonPlanePub = node.advertise<sensor_msgs::PointCloud2>(
		TOPIC_DEBUG_NON_PLANE_OUT, 1
	);
	debugPlanePub = node.advertise<sensor_msgs::PointCloud2>(
		TOPIC_DEBUG_PLANE_OUT, 1
	);
	anglePub = node.advertise<std_msgs::Float64>(TOPIC_OUT, 1);

	ros::spin();
}
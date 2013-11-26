#include <cmath>
#include <iostream>
#include <vector>
#include <pcl/common/centroid.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_labeled_clusters.h>
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
* Global variables
*/
Config config;
ros::Subscriber cloudSub;
ros::Publisher debugCroppedPub;
ros::Publisher debugNonPlanePub;
ros::Publisher debugPlanePub;
ros::Publisher anglePub;

/**
* Scaling down the cloud
* This should improve performance for future operations
*/
void scaleCloud(
	TheiaCloudPtr in,
	double inLeafSize,
	TheiaCloudPtr out
){
	pcl::VoxelGrid<TheiaPoint> grid;
	grid.setLeafSize(inLeafSize, inLeafSize, inLeafSize);

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
void filterPlanes(
	TheiaCloudPtr inCloud,
	TheiaCloudPtr outPlanes,
	TheiaCloudPtr outObjects
){
	TheiaCloudPtr workingCloudPtr(new TheiaCloud(*inCloud));
	size_t origCloudSize = workingCloudPtr->points.size();
	size_t minPlanePoints = config.minPercentage * origCloudSize;

	if(origCloudSize < 3){
		ROS_INFO("Input cloud for findPlanes is too small");
		ROS_INFO("Check cropping and rescaling parameters");
		return;
	}


	TheiaCloudPtr allPlanesCloudPtr(new TheiaCloud());
	PointIndices::Ptr inliers(new pcl::PointIndices());
	ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());

	// prepare plane segmentation
	SACSegmentation<TheiaPoint> seg;
	seg.setModelType(SACMODEL_PLANE);
	seg.setMethodType(SAC_RANSAC);
	seg.setMaxIterations(config.numbIterations);
	seg.setDistanceThreshold(config.planeDistThresh);
	seg.setOptimizeCoefficients(config.planeOptimize);

	// prepare extractor
	ExtractIndices<TheiaPoint> extract;

	do{
		seg.setInputCloud(workingCloudPtr);
		seg.segment(*inliers, *coefficients);

		/**
		* Stop the search for planes when the found plane
		* is too small (in terms of points).
		*/
		size_t numbInliers = inliers->indices.size();
		if(!numbInliers) break;
		if(numbInliers < minPlanePoints) break;
		
		// extract plane points
		TheiaCloudPtr planeCloudPtr(new TheiaCloud());
		extract.setInputCloud(workingCloudPtr);
		extract.setIndices(inliers);
		extract.setNegative(false);
		extract.filter(*planeCloudPtr);

		*allPlanesCloudPtr += *planeCloudPtr;

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
		workingCloudPtr->swap(*remainingCloudPtr);
	}while(true);

	*outPlanes = TheiaCloud(*allPlanesCloudPtr);
	*outObjects = TheiaCloud(*workingCloudPtr);
}

void initConfig(){
	ros::param::getCached("~config/leafSize", config.leafSize);
	ros::param::getCached("~config/minPercentage", config.minPercentage);
	ros::param::getCached("~config/objectSize", config.objectSize);
	ros::param::getCached("~config/numbIterations", config.numbIterations);
	ros::param::getCached("~config/planeDistThresh", config.planeDistThresh);
	ros::param::getCached("~config/planeOptimize", config.planeOptimize);
}

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr & rosMsgPtr){
	initConfig();

	TheiaCloudPtr cloudPtr(new TheiaCloud());
	pcl::fromROSMsg(*rosMsgPtr, *cloudPtr);

	TheiaCloudPtr scaledCloudPtr(new TheiaCloud());
	scaleCloud(cloudPtr, config.leafSize, scaledCloudPtr);
	cloudPtr.reset();

	TheiaCloudPtr planeCloudPtr(new TheiaCloud());
	TheiaCloudPtr objectCloudPtr(new TheiaCloud());
	filterPlanes(scaledCloudPtr, planeCloudPtr, objectCloudPtr);

	// debug
	visionCloudDebug(planeCloudPtr, debugPlanePub);
	visionCloudDebug(objectCloudPtr, debugNonPlanePub);
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
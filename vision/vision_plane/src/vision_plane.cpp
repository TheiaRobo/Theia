#include <cmath>
#include <iostream>
#include <vector>
#include <pcl16/common/centroid.h>
#include <pcl16/ModelCoefficients.h>
#include <pcl16/filters/extract_indices.h>
#include <pcl16/filters/voxel_grid.h>
#include <pcl16/sample_consensus/method_types.h>
#include <pcl16/sample_consensus/model_types.h>
#include <pcl16/segmentation/extract_clusters.h>
#include <pcl16/segmentation/sac_segmentation.h>
#include <pcl16/ros/conversions.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/PointCloud2.h>
#include <vision/cloud.h>
#include <vision_plane/Box.h>
#include <vision_plane/Boxes.h>

#include "vision_plane.h"

#define NODE_NAME "vision_plane"
#define TOPIC_IN "/camera/depth_registered/points"
#define TOPIC_OUT_BOX "/vision/plane/box"
#define TOPIC_DEBUG_CROPPED_OUT "/vision/plane/debug/cropped"
#define TOPIC_DEBUG_NON_PLANE_OUT "/vision/plane/debug/nonPlane"
#define TOPIC_DEBUG_PLANE_OUT "/vision/plane/debug/plane"

using namespace pcl16;
using namespace vision_plane;

/**
* Global variables
*/
Config config;
ros::Subscriber cloudSub;
ros::Publisher boxPub;
ros::Publisher debugCroppedPub;
ros::Publisher debugNonPlanePub;
ros::Publisher debugPlanePub;

int clusterToBox(
	TheiaCloudPtr inCloud,
	PointIndices & inIndices,
	Box & outBox
){
	int errorCode = 0;

	// find box
	Eigen::Vector4f minPoint;
	Eigen::Vector4f maxPoint;
	getMinMax3D(*inCloud, inIndices, minPoint, maxPoint);

	// build message
	Box box;
	box.minX = minPoint[0];
	box.maxX = maxPoint[0];
	box.minY = minPoint[1];
	box.maxY = maxPoint[1];
	box.minZ = minPoint[2];
	box.maxZ = maxPoint[2];

	outBox = box;

	return errorCode;
}

/**
* Scaling down the cloud
* This should improve performance for future operations
*/
void scaleCloud(
	TheiaCloudPtr in,
	double inLeafSize,
	TheiaCloudPtr out
){
	VoxelGrid<TheiaPoint> grid;
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
	PointIndices::Ptr inliers(new PointIndices());
	ModelCoefficients::Ptr coefficients(new ModelCoefficients());

	// prepare plane segmentation
	SACSegmentation<TheiaPoint> seg;
	seg.setModelType(SACMODEL_PLANE);
	seg.setMethodType(SAC_RANSAC);
	seg.setMaxIterations(config.numbIterations);
	seg.setDistanceThreshold(config.planeDistThresh);
	seg.setOptimizeCoefficients(config.planeOptimize);

	// prepare extractor
	ExtractIndices<TheiaPoint> extract;

	while(workingCloudPtr->points.size() >= 4){
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
	}

	*outPlanes = TheiaCloud(*allPlanesCloudPtr);
	*outObjects = TheiaCloud(*workingCloudPtr);
}

int findBoxes(
	TheiaCloudPtr inCloud,
	std::vector<Box> & outBoxVect
){
	int errorCode = 0;

	size_t numbPoints = inCloud->points.size();
	if(!numbPoints){
		std::cout << "Error in " << __FUNCTION__ << std::endl;
		std::cout << "Cloud is empty" << std::endl;
		return -1;
	}

	EuclideanClusterExtraction<TheiaPoint> extractor;
	extractor.setClusterTolerance(config.objectSize);
	extractor.setMinClusterSize(config.minClusterSize);
	extractor.setInputCloud(inCloud);

	std::vector<PointIndices> clusterVect;
	extractor.extract(clusterVect);

	size_t numbClusters = clusterVect.size();
	for(size_t i = 0; i < numbClusters; i++){
		PointIndices & cluster = clusterVect[i];
		
		Box box;
		errorCode = clusterToBox(inCloud, cluster, box);
		if(errorCode) return errorCode;
	
		outBoxVect.push_back(box);
	}

	return errorCode;
}

void initConfig(){
	ros::param::getCached("~config/leafSize", config.leafSize);
	ros::param::getCached("~config/minClusterSize", config.minClusterSize);
	ros::param::getCached("~config/minPercentage", config.minPercentage);
	ros::param::getCached("~config/objectSize", config.objectSize);
	ros::param::getCached("~config/numbIterations", config.numbIterations);
	ros::param::getCached("~config/planeDistThresh", config.planeDistThresh);
	ros::param::getCached("~config/planeOptimize", config.planeOptimize);
}

void publishBoxes(std::vector<Box> & inBoxVect){
	Boxes boxesMsg;
	boxesMsg.boxes = inBoxVect;
	boxPub.publish(boxesMsg);
}

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr & rosMsgPtr){
	initConfig();

	TheiaCloudPtr cloudPtr(new TheiaCloud());
	fromROSMsg(*rosMsgPtr, *cloudPtr);

	TheiaCloudPtr scaledCloudPtr(new TheiaCloud());
	scaleCloud(cloudPtr, config.leafSize, scaledCloudPtr);
	cloudPtr.reset();

	TheiaCloudPtr planeCloudPtr(new TheiaCloud());
	TheiaCloudPtr objectCloudPtr(new TheiaCloud());
	filterPlanes(scaledCloudPtr, planeCloudPtr, objectCloudPtr);

	std::vector<Box> boxVect;
	findBoxes(objectCloudPtr, boxVect);
	publishBoxes(boxVect);

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
	boxPub = node.advertise<Boxes>(TOPIC_OUT_BOX, 1);
	debugCroppedPub = node.advertise<sensor_msgs::PointCloud2>(
		TOPIC_DEBUG_CROPPED_OUT, 1
	);
	debugNonPlanePub = node.advertise<sensor_msgs::PointCloud2>(
		TOPIC_DEBUG_NON_PLANE_OUT, 1
	);
	debugPlanePub = node.advertise<sensor_msgs::PointCloud2>(
		TOPIC_DEBUG_PLANE_OUT, 1
	);

	ros::spin();
}
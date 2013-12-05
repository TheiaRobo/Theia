#ifndef VISION_COMMON_CLOUD
#define VISION_COMMON_CLOUD

// ROS includes
#include <ros/ros.h>

// PCL includes
#include <pcl16/point_cloud.h>
#include <pcl16/point_types.h>

typedef pcl16::PointXYZ TheiaPoint;
typedef pcl16::PointCloud<TheiaPoint> TheiaCloud;
typedef TheiaCloud::Ptr TheiaCloudPtr;

/**
* This function allows to easily remove boring points from the cloud.
*/
bool visionCloudCrop(
	TheiaCloudPtr in,
	TheiaCloudPtr out,
	double cuboid[3][2]
);

/**
* Send cloud to publisher.
* This function is lazy and only sends if a node is connected.
*/
void visionCloudDebug(
	TheiaCloudPtr cloudPtr,
	ros::Publisher & publisher
);

#endif
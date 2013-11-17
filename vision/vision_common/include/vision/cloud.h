#ifndef VISION_COMMON_CLOUD
#define VISION_COMMON_CLOUD

// ROS includes
#include <ros/ros.h>

// PCL includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

typedef pcl::PointXYZ TheiaPoint;
typedef pcl::PointCloud<TheiaPoint> TheiaCloud;
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
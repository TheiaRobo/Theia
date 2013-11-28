// ROS includes
#include <ros/ros.h>

// message
#include <sensor_msgs/PointCloud2.h>

// PCL includes
#include <pcl16/point_cloud.h>
#include <pcl16/point_types.h>
#include <pcl16/filters/crop_box.h>

#include <vision/cloud.h>

using namespace pcl16;

bool visionCloudCrop(
	TheiaCloudPtr in,
	TheiaCloudPtr out,
	double cuboid[3][2]
){
	/**
	* Preparation
	*/
	Eigen::Vector4f minDistanceVect;
	Eigen::Vector4f maxDistanceVect;

	for(int i = 0; i < 3; i++){
		if(cuboid[i][0] > cuboid[i][1]){
			minDistanceVect[i] = cuboid[i][1];
			maxDistanceVect[i] = cuboid[i][0];
		}else{
			minDistanceVect[i] = cuboid[i][0];
			maxDistanceVect[i] = cuboid[i][1];
		}
	}

	/**
	* Setup crop box
	*/
    CropBox<TheiaPoint> crop;
    crop.setMin(minDistanceVect);
    crop.setMax(maxDistanceVect);
    crop.setInputCloud(in);

    crop.filter(*out);
    
	return true;
}

void visionCloudDebug(
	TheiaCloudPtr cloudPtr,
	ros::Publisher & publisher
){
	if(!publisher.getNumSubscribers()) return;

	// create message from cloud
	sensor_msgs::PointCloud2 cloudMessage;
	toROSMsg(*cloudPtr, cloudMessage);

	/**
	* TODO
	*/
	cloudMessage.header.frame_id = "/camera_depth_optical_frame";

	publisher.publish(cloudMessage);
}
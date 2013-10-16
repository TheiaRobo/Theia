#ifndef VISION_COMMON_CLOUD
#define VISION_COMMON_CLOUD

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>

/**
* Type definitions
*/
typedef pcl::PointXYZ TheiaPoint;
typedef pcl::PointCloud<TheiaPoint> TheiaCloud;
typedef TheiaCloud::Ptr TheiaCloudPtr;

/**
* visionCloudCrop
* This function allows to easily remove boring points from the cloud.
*/
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
    pcl::CropBox<TheiaPoint> crop;
    crop.setMin(minDistanceVect);
    crop.setMax(maxDistanceVect);
    crop.setInputCloud(in);

    crop.filter(*out);
    
	return true;
}

#endif
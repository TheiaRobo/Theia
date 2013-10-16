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
	double min[3],
	double max[3]
){
	/**
	* Preparation
	*/
	Eigen::Vector3f translationVect;
	Eigen::Vector4f maxDistanceVect;

	for(int i = 0; i < 3; i++){
		translationVect[i] = (max[i] + min[i]) / 2;
		maxDistanceVect[i] = (max[i] - min[i]) / 2;
	}

	/**
	* Setup crop box
	*/
    pcl::CropBox<TheiaPoint> crop;
    crop.setTranslation(translationVect);
    crop.setMax(maxDistanceVect);
    crop.setInputCloud(in);

    crop.filter(*out);
    
	return true;
}

#endif
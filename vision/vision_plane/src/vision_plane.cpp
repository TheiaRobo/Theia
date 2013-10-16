// ROS includes
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

// PCL includes
#include <pcl/ModelCoefficients.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

// PCL ROS interface
#include <pcl/ros/conversions.h>

#define TOPIC_IN "/camera/depth_registered/points"

/**
* type definition
* for convenience
*/
typedef pcl::PointXYZ TheiaPoint;
typedef pcl::PointCloud<TheiaPoint> TheiaCloud;
typedef TheiaCloud::Ptr TheiaCloudPtr;

ros::Subscriber cloudSub;

/**
* We scale down the cloud.
* This should improve performance for future operations
*/
void scaleCloud(TheiaCloudPtr in, TheiaCloudPtr out, double leafSize){
    pcl::VoxelGrid<TheiaPoint> grid;
    grid.setInputCloud(in);
    grid.setLeafSize(leafSize, leafSize, leafSize);
    grid.filter(*out);
}

/**
* This function tries to find all planes in a point cloud.
* A RANSAC algorithm is used for this.
*
* Heavily inspired by
* http://www.pointclouds.org/documentation/tutorials/extract_indices.php
*/
void findPlanes(TheiaCloudPtr in, TheiaCloudPtr out){
    // config
    static const int numbIterations = 1000;
    static const double distanceThreshold = 0.01;
    static const bool optimizeCoefficients = true;

    // create the segmentation object
    pcl::SACSegmentation<TheiaPoint> seg;
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(numbIterations);
    seg.setDistanceThreshold(distanceThreshold);
    seg.setOptimizeCoefficients(optimizeCoefficients);

    /**
    * Finds
    * - Indices of points in plane
    * - Parameters of plane
    */
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());

    seg.setInputCloud(in);
    seg.segment(*inliers, *coefficients);

    /**
    * Extract plane from cloud
    */
    pcl::ExtractIndices<TheiaPoint> extract;
    extract.setNegative(false);
    extract.setIndices(inliers);
    extract.setInputCloud(in);
    extract.filter(*out);
}

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr & rosMsgPtr){
    // leaf size of voxel grid (in meters)
    static const double leafSize = 0.01;

    ROS_INFO("Cloud callback\n");
    ROS_INFO("- Extract cloud from ROS message\n");

    // extract cloud
    TheiaCloudPtr cloudPtr(new TheiaCloud());
    pcl::fromROSMsg(*rosMsgPtr, *cloudPtr);

    ROS_INFO("- Scale cloud\n");

    // scale cloud
    TheiaCloudPtr scaledCloudPtr(new TheiaCloud());
    scaleCloud(cloudPtr, scaledCloudPtr, leafSize);

    ROS_INFO("- Find planes in cloud\n");

    // find planes
    TheiaCloudPtr planeCloudPtr(new TheiaCloud());
    findPlanes(scaledCloudPtr, planeCloudPtr);

    ROS_INFO("- End\n");
}

int main (int argc, char ** argv){
    // init ROS
    ros::init(argc, argv, "vision_plane");
    ros::NodeHandle node;

    // register handler for point clouds
    cloudSub = node.subscribe(TOPIC_IN, 1, cloudCallback);

    // run main loop
    ros::spin();
}
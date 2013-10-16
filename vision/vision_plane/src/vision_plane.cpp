// ROS includes
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

// PCL includes
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

// PCL ROS interface
#include <pcl/ros/conversions.h>

// Theia includes
#include <vision/cloud.h>

#define TOPIC_IN "/camera/depth_registered/points"
#define TOPIC_OUT "/vision/plane"

/**
* type definition
* for convenience
*/
ros::Subscriber cloudSub;
ros::Publisher planePub;

/**
* Configuration options
*/
static double cropMinDist[3];
static double cropMaxDist[3];
static int planeNumbIterations;
static double planeDistanceThreshold;
static bool planeOptimize;

/**
* We scale down the cloud.
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
void findPlanes(TheiaCloudPtr in, TheiaCloudPtr out){
    // create the segmentation object
    pcl::SACSegmentation<TheiaPoint> seg;
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(planeNumbIterations);
    seg.setDistanceThreshold(planeDistanceThreshold);
    seg.setOptimizeCoefficients(planeOptimize);

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
    ROS_INFO("Cloud callback");

    /**
    * Extract cloud from message
    */
    ROS_INFO("- Extract cloud from ROS message");

    TheiaCloudPtr cloudPtr(new TheiaCloud());
    pcl::fromROSMsg(*rosMsgPtr, *cloudPtr);

    /**
    * Crop cloud
    */
    ROS_INFO("- Crop cloud");

    TheiaCloudPtr croppedCloudPtr(new TheiaCloud());
    visionCloudCrop(cloudPtr, croppedCloudPtr, cropMinDist, cropMaxDist);

    sensor_msgs::PointCloud2 outMsg;
    pcl::toROSMsg(*croppedCloudPtr, outMsg);

    planePub.publish(outMsg);

    cloudPtr.reset();

    /**
    * Change resolution
    */
    ROS_INFO("- Rescale cloud");

    TheiaCloudPtr scaledCloudPtr(new TheiaCloud());
    scaleCloud(croppedCloudPtr, scaledCloudPtr);

    croppedCloudPtr.reset();

    /**
    * Find planes in cloud
    */
    ROS_INFO("- Find planes");

    TheiaCloudPtr planeCloudPtr(new TheiaCloud());
    findPlanes(scaledCloudPtr, planeCloudPtr);

    scaledCloudPtr.reset();

    /**
    * Send output message
    */
    ROS_INFO("- Send message");

    // sensor_msgs::PointCloud2 outMsg;
    // pcl::toROSMsg(*planeCloudPtr, outMsg);

    planePub.publish(outMsg);
    planeCloudPtr.reset();

    ROS_INFO("- End");
}

void initConfig(){
    cropMinDist[0] = -5;
    cropMinDist[1] = -5;
    cropMinDist[2] = -5;

    cropMaxDist[0] = +5;
    cropMaxDist[1] = +5;
    cropMaxDist[2] = +5;

    planeNumbIterations = 1000;
    planeDistanceThreshold = 0.01;
    planeOptimize = true;
}

int main (int argc, char ** argv){
    initConfig();

    // init ROS
    ros::init(argc, argv, "vision_plane");
    ros::NodeHandle node;

    // setup input and output topics
    cloudSub = node.subscribe(TOPIC_IN, 1, cloudCallback);
    planePub = node.advertise<sensor_msgs::PointCloud2>(TOPIC_OUT, 1);

    // run main loop
    ros::spin();
}
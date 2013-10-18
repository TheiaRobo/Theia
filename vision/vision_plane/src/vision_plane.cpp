#include <cmath>
#include <vector>

// ROS includes
#include <ros/ros.h>
#include <std_msgs/Float64.h>
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

#define NODE_NAME "vision_plane"
#define TOPIC_IN "/camera/depth_registered/points"
#define TOPIC_OUT "/vision/plane/angle"
#define TOPIC_DEBUG_CROPPED_OUT "/vision/plane/debug/cropped"
#define TOPIC_DEBUG_PLANE_OUT "/vision/plane/debug/plane"

/**
* Type definitions
*/
typedef struct {
    double normal[3];
    double centroid[3];
    double area;
} Plane_t;

/**
* Global variables
*/
ros::Subscriber cloudSub;
ros::Publisher debugCroppedPub;
ros::Publisher debugPlanePub;
ros::Publisher anglePub;

/**
* Configuration options
*
* TODO:
* - Replace by cached parameters
*/
int planeNumbIterations;
double planeDistanceThreshold;
bool planeOptimize;

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
void findPlanes(TheiaCloudPtr in, std::vector<Plane_t> & planeVect){
    double minPercentage;
    ros::param::getCached("~config/plane/percentage", minPercentage);

    // for debugging
    TheiaCloudPtr allPlanesCloudPtr(new TheiaCloud());

    /**
    * RANSAC needs at least three points
    */
    int origCloudSize = in->points.size();

    if(origCloudSize < 3){
        ROS_INFO("Input cloud for findPlanes is too small");
        ROS_INFO("Check cropping and rescaling parameters");
        return;
    }

    // create the segmentation object
    pcl::SACSegmentation<TheiaPoint> seg;
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(planeNumbIterations);
    seg.setDistanceThreshold(planeDistanceThreshold);
    seg.setOptimizeCoefficients(planeOptimize);

    // contains the indices of points in plane
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

    // contains the parameters for the hessian normal equation
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());

    do{
        seg.setInputCloud(in);
        seg.segment(*inliers, *coefficients);

        /**
        * Stop the search for planes when the found plane
        * is too small (in terms of points).
        */
        int numbInliers = inliers->indices.size();

        if(
            numbInliers == 0
            || numbInliers < minPercentage * origCloudSize
        ){
            break;
        }

        Plane_t plane;
        
        /**
        * Find normal
        */
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

        planeCloudPtr.reset();

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
}

/**
* This function takes a a vector of planes
* and calculates an angle for it.
*/
double planeVectToAngle(std::vector<Plane_t> & planeVect){
    double totalArea = 0;
    double totalAngle = 0;
    for(int i = 0; i < planeVect.size(); i++){
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

        ROS_INFO("Plane #%d, Angle %f", i, angle);
    }

    double averageAngle = totalAngle / totalArea;
    return averageAngle;
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
    cropCloud(cloudPtr, croppedCloudPtr);

    // debug
    visionCloudDebug(croppedCloudPtr, debugCroppedPub);

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

    std::vector<Plane_t> planeVect;
    findPlanes(scaledCloudPtr, planeVect);

    scaledCloudPtr.reset();

    if(planeVect.empty()){
        return;
    }

    /**
    * Estimate angle to wall
    */
    double angle = planeVectToAngle(planeVect);

    /**
    * Send output message
    */
    ROS_INFO("- Send message");
    
    std_msgs::Float64 outMsg;
    outMsg.data = 180 / M_PI * angle;

    anglePub.publish(outMsg);

    ROS_INFO("- End");
}

void initConfig(){
    planeNumbIterations = 1000;
    planeDistanceThreshold = 0.01;
    planeOptimize = true;
}

int main (int argc, char ** argv){
    initConfig();

    // init ROS
    ros::init(argc, argv, NODE_NAME);
    ros::NodeHandle node;

    /**
    * Subscribers
    */
    cloudSub = node.subscribe(TOPIC_IN, 1, cloudCallback);

    /**
    * Publishers
    */
    debugCroppedPub = node.advertise<sensor_msgs::PointCloud2>(
        TOPIC_DEBUG_CROPPED_OUT, 1
    );

    debugPlanePub = node.advertise<sensor_msgs::PointCloud2>(
        TOPIC_DEBUG_PLANE_OUT, 1
    );

    anglePub = node.advertise<std_msgs::Float64>(TOPIC_OUT, 1);

    // run main loop
    ros::spin();
}
// ROS includes
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

// PCL includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

#define TOPIC_IN "/camera/depth_registered/points"


ros::Subscriber cloudSub;

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud){
    // leaf size of voxel grid (in meters)
    double leafSize = 0.01;
    
    /**
    * sample down point cloud
    */
    sensor_msgs::PointCloud2 filtered;
    pcl::VoxelGrid<sensor_msgs::PointCloud2> grid;
    grid.setInputCloud(cloud);
    grid.setLeafSize(leafSize, leafSize, leafSize);
    grid.filter(filtered);
}

int main (int argc, char ** argv){
    // init ROS
    ros::init(argc, argv, "vision_plane");
    ros::NodeHandle node;

    // register handler for point clouds
    cloudSub = node.subscribe("/camera/depth", 1, cloudCallback);

    // run main loop
    ros::spin();
}
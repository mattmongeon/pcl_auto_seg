#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>

#include <pcl_ros/point_cloud.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <iostream>


void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
    // We are simply going to save a point cloud to file for now.
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*input, *cloud);

    pcl::io::savePCDFileASCII("/home/mongeon/point_cloud.pcd", *cloud);
	std::cerr << "File saved!" << std::endl;
	
    exit(0);
}

int main(int argc, char** argv)
{
    // Initialize ROS
    ros::init(argc, argv, "pcl_auto_seg");
    ros::NodeHandle nh;

    // Subscriber for taking in the input point cloud.
    ros::Subscriber sub = nh.subscribe("camera/depth_registered/points", 1, cloud_cb);

    ros::spin();

    return 0;
}


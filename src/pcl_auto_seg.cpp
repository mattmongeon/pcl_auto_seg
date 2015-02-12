#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>

#include <pcl_ros/point_cloud.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input)
{
	// Read in the point cloud from the ROS message.
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>() );
    pcl::fromROSMsg(*input, *cloud);

	// Create the normal estimation class, and pass the input dataset to it
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud(cloud);

	// Create an empty kdtree representation, and pass it to the normal estimation
	// object.  Its contents will be filed inside the object, based on the given
	// input dataset (as no other search surface is given).
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree( new pcl::search::KdTree<pcl::PointXYZ>() );
	ne.setSearchMethod(tree);

	// Output datasets
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals( new pcl::PointCloud<pcl::Normal>() );

	// Use all neighbors in a sphere of radius 3 cm
	ne.setRadiusSearch(0.03);

	// Compute the features
	ne.compute( *cloud_normals );
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


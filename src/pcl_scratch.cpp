#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/Vertices.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/segmentation/impl/conditional_euclidean_clustering.hpp>

#include <pcl/filters/extract_indices.h>

#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>

#include <pcl/ModelCoefficients.h>

#include <pcl/surface/mls.h>
#include <pcl/surface/concave_hull.h>

#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>

#include <boost/thread/thread.hpp>

ros::Publisher pub;

float deg2rad(float alpha)
{
    return (alpha * 0.017453293f);
}

void passthrough_z(const sensor_msgs::PointCloud2ConstPtr& input, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_passthrough)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*input, *cloud);

    // Create the filtering object
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud (boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB> >(*cloud));
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.0, 6.0);
    pass.filter (*cloud_passthrough);
}

void passthrough_y(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_passthrough)
{
    // Create the filtering object
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud (boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB> >(*cloud_passthrough));
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (0.0, 5.0);
    pass.filter (*cloud_passthrough);
}

void passthrough_x(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_passthrough)
{
    // Create the filtering object
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud (boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB> >(*cloud_passthrough));
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (-2.0, 2.0);
    pass.filter (*cloud_passthrough);
}

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
    // Do data processing here...
  
    // run pass through filter to shrink point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_passthrough(new pcl::PointCloud<pcl::PointXYZRGB>);
    passthrough_z(input, cloud_passthrough);
    passthrough_y(cloud_passthrough);
    passthrough_x(cloud_passthrough);
    pub.publish(*cloud_passthrough);

    // run ransac to find floor
    //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZRGB>);
    //ransac(cloud_passthrough, cloud_projected);
    //pub.publish(*cloud_projected);
}

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "pcl_node");
    ros::NodeHandle nh;

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe ("camera/depth_registered/points", 1, cloud_cb);

    // Create a ROS publisher for the output point cloud
    pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

    // Spin
    ros::spin();
}

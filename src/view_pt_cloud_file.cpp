#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <iostream>
#include <string>


void visualize(pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud)
{
    //init visualizer
	pcl::visualization::PCLVisualizer::Ptr visualizer_o_Ptr (new pcl::visualization::PCLVisualizer());
	visualizer_o_Ptr->setSize(640, 480);
	visualizer_o_Ptr->setPosition(640, 0);
	visualizer_o_Ptr->setBackgroundColor(0x00, 0x00, 0x00);
	visualizer_o_Ptr->addCoordinateSystem(1.0);
	visualizer_o_Ptr->initCameraParameters();
	visualizer_o_Ptr->addPointCloud(pCloud, "cloud");

	//reload visualizer content
	visualizer_o_Ptr->spinOnce(10000000);
}


void view_file( const std::string& fileName )
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr pScene( new pcl::PointCloud<pcl::PointXYZ>() );

	if (pcl::io::loadPCDFile( fileName, *pScene ) < 0)
	{
		std::cout << "Error loading scene cloud." << std::endl;
		return;
	}

	visualize( pScene );
}


int main(int argc, char** argv)
{
    // Initialize ROS
    ros::init(argc, argv, "view_pt_cloud_file");
    ros::NodeHandle nh;

	std::string fileName;

	ros::param::get("~file_name", fileName);
	view_file( fileName );

    ros::spin();

    return 0;
}


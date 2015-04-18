#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <geometry_msgs/Vector3.h>

#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <vector>
#include <iostream>


// --- Declarations --- //

ros::Publisher pub;
pcl::visualization::PCLVisualizer::Ptr visualizer_o_Ptr;

typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudPtr;
typedef pcl::PointCloud<pcl::Normal>::Ptr NormalCloudPtr;

int controllerState = 0;


void visualize(pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud, pcl::visualization::PCLVisualizer::Ptr pVisualizer)
{
    //init visualizer
	pVisualizer->setSize(640, 480);
	pVisualizer->setPosition(640, 0);
	pVisualizer->setBackgroundColor(0x00, 0x00, 0x00);
	pVisualizer->initCameraParameters();
	pVisualizer->addPointCloud(pCloud, "cloud");

	//reload visualizer content
	pVisualizer->spinOnce(1);
}


// Align a collection of object templates to a sample point cloud
void cloud_cb( const sensor_msgs::PointCloud2ConstPtr& input )
{
  if( controllerState != 1 )
    return;

    //--- Convert Incoming Cloud --- //

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud( new pcl::PointCloud<pcl::PointXYZ> );
    pcl::fromROSMsg( *input, *cloud );

	
	// --- Z-Filter And Downsample Cloud --- //

	// Preprocess the cloud by removing distant points
	const float depth_limit = 1.5;
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud( cloud );
	pass.setFilterFieldName( "z" );
	pass.setFilterLimits( 0, 1.5 );
	pass.filter( *cloud );

	// It is possible to not have any points after z-filtering (ex. if we are looking up).
	// Just bail if there is nothing left.
	if( cloud->points.size() == 0 )
		return;

	
	// --- Calculate Scene Normals --- //

	pcl::PointCloud<pcl::Normal>::Ptr pSceneNormals( new pcl::PointCloud<pcl::Normal>() );
	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> normEst;
	normEst.setKSearch(10);
	normEst.setInputCloud( cloud );
	normEst.compute( *pSceneNormals );


	// --- Extract Floor --- //

	pcl::PointIndices::Ptr inliers_plane( new pcl::PointIndices );
	pcl::ModelCoefficients::Ptr coefficients_plane( new pcl::ModelCoefficients );

	pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg1; 
	seg1.setOptimizeCoefficients( true );
	seg1.setModelType( pcl::SACMODEL_NORMAL_PLANE );
	seg1.setNormalDistanceWeight( 0.05 );
	seg1.setMethodType( pcl::SAC_RANSAC );
	seg1.setMaxIterations( 100 );
	seg1.setDistanceThreshold( 0.075 );
	seg1.setInputCloud( cloud );
	seg1.setInputNormals( pSceneNormals );
	// Obtain the plane inliers and coefficients
	seg1.segment( *inliers_plane, *coefficients_plane );

	// Extract the planar inliers from the input cloud
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	extract.setInputCloud( cloud );
	extract.setIndices( inliers_plane );
	extract.setNegative( false );

	// Write the planar inliers to disk
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane( new pcl::PointCloud<pcl::PointXYZ> );
	extract.filter( *cloud_plane );


	// --- Publish Floor Normal --- //

    geometry_msgs::Vector3 floorNorm;
	floorNorm.x = coefficients_plane->values[0];
	floorNorm.y = coefficients_plane->values[1];
	floorNorm.z = coefficients_plane->values[2];
	
	pub.publish(floorNorm);
}


void current_state_cb( const std_msgs::Int32& state )
{
  controllerState = state.data;
}


int main(int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "floor_normal_pub");
    ros::NodeHandle nh;

    // Create a ROS publisher for the normal of the floor relative to the ASUS.
    pub = nh.advertise<geometry_msgs::Vector3>("/floor_normal", 1);

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe ("camera/depth_registered/points", 1, cloud_cb);

    // Create a subscriber for the current state
    ros::Subscriber stateSub = nh.subscribe("/control_current_state", 1, current_state_cb );


	// Create visualizer
	//visualizer_o_Ptr = pcl::visualization::PCLVisualizer::Ptr(new pcl::visualization::PCLVisualizer());
	
    // Spin
    ros::spin();
}

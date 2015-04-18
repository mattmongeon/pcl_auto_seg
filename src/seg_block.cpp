#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <geometry_msgs/Pose.h>

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

#include "pcl_auto_seg/feature_cloud.h"
#include "pcl_auto_seg/template_alignment.h"

#include <vector>
#include <iostream>


// --- Declarations --- //

ros::Publisher pub;
ros::Publisher cloud_pub;
pcl::visualization::PCLVisualizer::Ptr visualizer_o_Ptr;

typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudPtr;
typedef pcl::PointCloud<pcl::Normal>::Ptr NormalCloudPtr;
typedef std::pair<PointCloudPtr, NormalCloudPtr> ModelPair;
std::vector<ModelPair> models;

std::vector<FeatureCloud> object_templates;
TemplateAlignment template_align;

int controllerState = 0;


////////////////////////////////////////////////////////////////////////////////
// From the tutorial "Aligning Object Templates to a Point Cloud", modified
// for purposes particular to this project.
////////////////////////////////////////////////////////////////////////////////

void visualize(pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud, pcl::visualization::PCLVisualizer::Ptr pVisualizer)
{
    //init visualizer
	pVisualizer->setSize(640, 480);
	pVisualizer->setPosition(640, 0);
	pVisualizer->setBackgroundColor(0x00, 0x00, 0x00);
	pVisualizer->initCameraParameters();
	pVisualizer->addPointCloud(pCloud, "cloud");

	//reload visualizer content
	pVisualizer->spinOnce(1000000);
}

void visualize(pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr pBlock, pcl::visualization::PCLVisualizer::Ptr pVisualizer)
{
    //init visualizer
	pVisualizer->setSize(640, 480);
	pVisualizer->setPosition(640, 0);
	pVisualizer->setBackgroundColor(0x00, 0x00, 0x00);
	pVisualizer->initCameraParameters();
	pVisualizer->addPointCloud(pCloud, "cloud");
	pVisualizer->addPointCloud(pBlock, "block");

	//reload visualizer content
	pVisualizer->spinOnce(1000000);
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
	pcl::PassThrough<pcl::PointXYZ> pass_z;
	pass_z.setInputCloud( cloud );
	pass_z.setFilterFieldName( "z" );
	pass_z.setFilterLimits( 0, 1.75 );
	pass_z.filter( *cloud );

	pcl::PassThrough<pcl::PointXYZ> pass_y;
	pass_y.setInputCloud( cloud );
	pass_y.setFilterFieldName("y");
	pass_y.setFilterLimits( -0.5, 0.2 );
	pass_y.filter( *cloud );
	
	pcl::PassThrough<pcl::PointXYZ> pass_x;
	pass_x.setInputCloud( cloud );
	pass_x.setFilterFieldName("x");
	pass_x.setFilterLimits( -0.5, 0.5 );
	pass_x.filter( *cloud );

	// It is possible to not have any points after z-filtering (ex. if we are looking up).
	// Just bail if there is nothing left.
	if( cloud->points.size() == 0 )
		return;

	//visualize( cloud, visualizer_o_Ptr );

	
	// --- Calculate Scene Normals --- //

	pcl::PointCloud<pcl::Normal>::Ptr pSceneNormals( new pcl::PointCloud<pcl::Normal>() );
	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> normEst;
	normEst.setKSearch(10);
	normEst.setInputCloud( cloud );
	normEst.compute( *pSceneNormals );


	// --- Get Rid Of Floor --- //

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

	// Remove the planar inliers, extract the rest
	pcl::PointCloud<pcl::PointXYZ>::Ptr filteredScene( new pcl::PointCloud<pcl::PointXYZ> );
	extract.setNegative( true );
	extract.filter( *filteredScene );

	pcl::ExtractIndices<pcl::Normal> extract_normals;
	pcl::PointCloud<pcl::Normal>::Ptr filteredSceneNormals( new pcl::PointCloud<pcl::Normal> );
	extract_normals.setNegative( true );
	extract_normals.setInputCloud( pSceneNormals );
	extract_normals.setIndices( inliers_plane );
	extract_normals.filter( *filteredSceneNormals );	

	if( filteredScene->points.size() == 0 )
		return;
	
	
	// --- Set Our Target Cloud --- //

	// Assign to the target FeatureCloud
	FeatureCloud target_cloud;
	target_cloud.setInputCloud( filteredScene );


	// --- Visualize the Filtered Cloud --- //

	//visualize( filteredScene, visualizer_o_Ptr );


	// --- Set Input Cloud For Alignment --- //
	
	template_align.setTargetCloud( target_cloud );
	

	// --- Align Templates --- //

	std::cout << "Searching for best fit" << std::endl;
	// Find the best template alignment
	TemplateAlignment::Result best_alignment;
	int best_index = template_align.findBestAlignment( best_alignment );
	std::cerr << "Best alignment index:  " << best_index << std::endl;
	const FeatureCloud &best_template = object_templates[best_index];


	// --- Report Best Match --- //
	
	// Print the alignment fitness score (values less than 0.00002 are good)
	std::cerr << "Best fitness score: " << best_alignment.fitness_score << std::endl;

	// Print the rotation matrix and translation vector
	Eigen::Matrix3f rotation = best_alignment.final_transformation.block<3,3>(0, 0);
	Eigen::Vector3f translation = best_alignment.final_transformation.block<3,1>(0, 3);

	std::cerr << std::setprecision(3);
	std::cerr << std::endl;
	std::cerr << "    | " << rotation(0,0) << " " << rotation(0,1) << " " << rotation(0,2) << " | " << std::endl;
	std::cerr << "R = | " << rotation(1,0) << " " << rotation(1,1) << " " << rotation(1,2) << " | " << std::endl;
	std::cerr << "    | " << rotation(2,0) << " " << rotation(2,1) << " " << rotation(2,2) << " | " << std::endl;
	std::cerr << std::endl;
	std::cerr << "t = < " << translation(0) << ", " << translation(1) << ", " << translation(2) << " >" << std::endl << std::endl;


	// pcl::PointCloud<pcl::PointXYZ> transformedCloud;
	// pcl::transformPointCloud( *best_template.getPointCloud(), transformedCloud, best_alignment.final_transformation);
	// visualize( filteredScene, transformedCloud.makeShared(), visualizer_o_Ptr );
	

	// --- Publish --- //

	// TODO:  Clean up this part.
    geometry_msgs::Pose pose;

	tf::Matrix3x3 rot( rotation(0,0), rotation(0,1), rotation(0,2),
					   rotation(1,0), rotation(1,1), rotation(1,2),
					   rotation(2,0), rotation(2,1), rotation(2,2) );
	tf::Quaternion q;
	rot.getRotation(q);
	pose.orientation.w = q.getW();
	pose.orientation.x = q.getX();
	pose.orientation.y = q.getY();
	pose.orientation.z = q.getZ();

	tf::Vector3 t( translation(0), translation(1), translation(2) );
	pose.position.x = t.getX();
	pose.position.y = t.getY();
	pose.position.z = t.getZ();
	
	std::cerr << "Publishing" << std::endl;
	pub.publish(pose);

	sensor_msgs::PointCloud2 toPub;
    pcl::toROSMsg( *filteredScene, toPub );
	cloud_pub.publish(toPub);
}


void current_state_cb( const std_msgs::Int32& state )
{
  controllerState = state.data;
}


int main(int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "seg_block");
    ros::NodeHandle nh;

	// --- Set Up Template Container --- //
	
	// Load the object templates specified in the object_templates.txt file
	object_templates.clear();

	FeatureCloud template_cloud1;
	template_cloud1.makeCube(0.03);
	object_templates.push_back(template_cloud1);

	FeatureCloud template_cloud2;
	template_cloud2.makeCube(0.04);
	object_templates.push_back(template_cloud2);
	
	FeatureCloud template_cloud3;
	template_cloud3.makeCube(0.045);
	object_templates.push_back(template_cloud3);

	FeatureCloud template_cloud4;
	template_cloud4.makeCube(0.05);
	object_templates.push_back(template_cloud4);

	FeatureCloud template_cloud5;
	template_cloud5.makeCube(0.055);
	object_templates.push_back(template_cloud5);

	for (size_t i = 0; i < object_templates.size (); ++i)
	{
		template_align.addTemplateCloud (object_templates[i]);
	}

	
	// Create visualizer
	//visualizer_o_Ptr = pcl::visualization::PCLVisualizer::Ptr(new pcl::visualization::PCLVisualizer());

    // Create a ROS publisher for the pose of the block relative to the ASUS.
    pub = nh.advertise<geometry_msgs::Pose>("/block_pose", 1);
	cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/filtered_cloud", 1);

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe ("camera/depth_registered/points", 1, cloud_cb);
    
    // Create a subscriber for the current state
    ros::Subscriber stateSub = nh.subscribe("/control_current_state", 1, current_state_cb );

    // Spin
    ros::spin();
}

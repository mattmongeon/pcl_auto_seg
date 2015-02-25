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

#include <pcl/recognition/ransac_based/obj_rec_ransac.h>


// --- From Template Matching --- //

#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>

#include <pcl_ros/point_cloud.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>

//#include <pcl/point_cloud.h>
#include <pcl/correspondence.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/registration/sample_consensus_prerejective.h>

#include <iostream>
#include <list>
#include <vector>
#include <utility>


ros::Publisher pub;
pcl::PointCloud<pcl::PointXYZ>::Ptr pCube;
pcl::PointCloud<pcl::Normal>::Ptr pCubeNormals;
pcl::PointCloud<pcl::Normal>::Ptr pSceneNormals;

typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudPtr;
typedef pcl::PointCloud<pcl::Normal>::Ptr NormalCloudPtr;
typedef std::pair<PointCloudPtr, NormalCloudPtr> ModelPair;
std::vector<ModelPair> models;


void makeCube(float side_m)
{
	// --- Define Constants --- //

	// const float cubeShortSide = 0.051;  // Horizontal side length (m)
	// const float cubeLongSide = 0.051;   // vertical side length (m)
	float cubeShortSide = side_m;  // Horizontal side length (m)
	float cubeLongSide = side_m;   // vertical side length (m)

	// How many points on a long side (esitmate) for making test cubes, either
	// for ICP registration or for testing fit_planes.cpp
	const int nLongSideHighRes = 25;
	const int nShortSideHighRes = 25;

	
	// --- Generate Cube Model Point Cloud --- //

	int npointsBottomFace = nShortSideHighRes * nShortSideHighRes;
	int npointsSideFace = nShortSideHighRes * nLongSideHighRes;
	int npointsTotal = npointsBottomFace + 2 * npointsSideFace;
	
	float dxShortSide = cubeShortSide / (float)nShortSideHighRes;
	float dxLongSide =  cubeLongSide /  (float)nLongSideHighRes;
	
	std::cerr << "# dxShortSide: " << dxShortSide << std::endl;
	std::cerr << "# dxLongSide: " << dxLongSide << std::endl;
	std::cerr << "# npointsTotal: " << npointsTotal << std::endl;

	pcl::PointCloud<pcl::PointXYZ>::Ptr model( new pcl::PointCloud<pcl::PointXYZ>() );
	model->width = npointsTotal;
	model->height = 1;
	//model->points.resize(npointsTotal); // allocate space for all the points we need
	
	// make the top and bottom cap faces
	// these go at y = +- cubeLongSide /2 
	// from x,z = -ls/2, -ls/2  to x,z = ls/2, ls/2
	int counter = 0;
	float xval, yval, zval;
	float xOffset, yOffset, zOffset;

	/*
	// top face
	yval = cubeLongSide / 2;
	xOffset = - cubeShortSide / 2;
	zOffset = - cubeShortSide / 2;
	for(int i = 0; i < nShortSideHighRes; i++){
		for(int j = 0; j < nShortSideHighRes; j++){
			model->points.push_back(pcl::PointXYZ());
			model->points[counter].x = i*dxShortSide +  xOffset;
			model->points[counter].y = yval;
			model->points[counter].z = j*dxShortSide + zOffset;
			counter++;
		}
	}
	*/

	// bottom face
	yval = -cubeLongSide / 2;
	xOffset = - cubeShortSide / 2;
	zOffset = - cubeShortSide / 2;
	for(int i = 0; i < nShortSideHighRes; i++){
		for(int j = 0; j < nShortSideHighRes; j++){
			model->points.push_back(pcl::PointXYZ());
			model->points[counter].x = i*dxShortSide + xOffset;
			model->points[counter].y = yval;
			model->points[counter].z = j*dxShortSide + zOffset;
			counter++;
		}
	}

	// make each side plane
	// 1) z= -cubeShortSide/2, x: from - cubeShortSide/2 to cubeShortSide/2, y from: -cubeLongSide/2, to cubeLongSide/2
	// 2) z= cubeShortSide/2, x: from - cubeShortSide/2 to cubeShortSide/2, y from: -cubeLongSide/2, to cubeLongSide/2
	// 3) x = -cubeShortSide/2, z: from - cubeShortSide/2 to cubeShortSide/2, y from: -cubeLongSide/2, to cubeLongSide/2
	// 4) x = -cubeShortSide/2, z: from - cubeShortSide/2 to cubeShortSide/2, y from: -cubeLongSide/2, to cubeLongSide/2


	zval = -cubeShortSide / 2;
	xOffset = - cubeShortSide / 2;
	yOffset = - cubeLongSide / 2;
	for(int i = 0; i < nShortSideHighRes; i++){
		for(int j = 0; j < nLongSideHighRes; j++){
			model->points.push_back(pcl::PointXYZ());
			model->points[counter].x = i*dxShortSide + xOffset;
			model->points[counter].y = j*dxLongSide + yOffset;
			model->points[counter].z = zval;
			counter++;
		}
	}

	/*
	zval = cubeShortSide / 2;
	xOffset = - cubeShortSide / 2;
	yOffset = - cubeLongSide / 2;
	for(int i = 0; i < nShortSideHighRes; i++){
		for(int j = 0; j < nLongSideHighRes; j++){
			model->points[counter].x = i*dxShortSide + xOffset;
			model->points[counter].y = j*dxLongSide +  yOffset;
			model->points[counter].z = zval;
			counter++;
		}
	}

	xval = -cubeShortSide / 2;
	zOffset = - cubeShortSide / 2;
	yOffset = - cubeLongSide / 2;
	for(int i = 0; i < nShortSideHighRes; i++){
		for(int j = 0; j < nLongSideHighRes; j++){
			model->points[counter].x = xval;
			model->points[counter].y = j*dxLongSide + yOffset;
			model->points[counter].z = i*dxShortSide + zOffset;
			counter++;
		}
	}
	*/
	
	xval = cubeShortSide / 2;
	zOffset = - cubeShortSide / 2;
	yOffset = - cubeLongSide / 2;
	for(int i = 0; i < nShortSideHighRes; i++){
		for(int j = 0; j < nLongSideHighRes; j++){
			model->points.push_back(pcl::PointXYZ());
			model->points[counter].x = xval;
			model->points[counter].y = j*dxLongSide + yOffset;
			model->points[counter].z = i*dxShortSide + zOffset;
			counter++;
		}
	}


	// --- Compute Normals --- //

	NormalCloudPtr normals( new pcl::PointCloud<pcl::Normal> );
	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> mNormEst;
	mNormEst.setKSearch(25);
	mNormEst.setInputCloud( model );
	mNormEst.compute( *normals );


	// --- Add It To The Library --- //

	models.push_back( ModelPair(model, normals) );
}

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

	for( std::size_t i = 0; i < models.size(); ++i )
	{
		std::stringstream name;
		name << "cube" << i;
		visualizer_o_Ptr->addPointCloud(models[i].first, name.str());
	}
	
	//reload visualizer content
	visualizer_o_Ptr->spinOnce(10000000);
}

template<typename T>
pcl::PointCloud<pcl::PointXYZ> convertToXYZ( T& cloud )
{
	pcl::PointCloud<pcl::PointXYZ> tempCloud;
	pcl::copyPointCloud(cloud, tempCloud);
	return tempCloud;
}

float deg2rad(float alpha)
{
    return (alpha * 0.017453293f);
}

void passthrough_z(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_passthrough)
{
    // Create the filtering object
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud( boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >(*cloud_passthrough) );
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.0, 2.0);
    pass.filter(*cloud_passthrough);
}

void passthrough_z(const sensor_msgs::PointCloud2ConstPtr& input, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_passthrough)
{
    pcl::fromROSMsg(*input, *cloud_passthrough);

	passthrough_z(cloud_passthrough);
}

void passthrough_y(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_passthrough)
{
    // Create the filtering object
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >(*cloud_passthrough));
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (0.0, 5.0);
    pass.filter(*cloud_passthrough);
}

void passthrough_x(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_passthrough)
{
    // Create the filtering object
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >(*cloud_passthrough));
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (-2.0, 2.0);
    pass.filter(*cloud_passthrough);
}

void processImage(pcl::PointCloud<pcl::PointXYZ>::Ptr pScene)
{
	// --- Compute Normals --- //
		
	pSceneNormals.reset( new pcl::PointCloud<pcl::Normal>() );
	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> mNormEst;
	mNormEst.setKSearch(10);
	mNormEst.setInputCloud( pScene );
	mNormEst.compute( *pSceneNormals );

	std::cout << "Scene normals computed." << std::endl;
	std::cout << "Size of normal cloud:  " << pSceneNormals->size() << std::endl;
	std::cout << "Width of normal cloud:  " << pSceneNormals->width << std::endl;
	std::cout << "Height of normal cloud:  " << pSceneNormals->height << std::endl;


	// --- Downsample Scene Cloud --- //

	pcl::PointCloud<int> sampled_indices;

	float scene_ss_ = 0.01f;

	pcl::UniformSampling<pcl::PointXYZ> mUniformSampling;
	mUniformSampling.setInputCloud( pScene );
	mUniformSampling.setRadiusSearch( scene_ss_ );
	mUniformSampling.compute( sampled_indices );
	pcl::PointCloud<pcl::PointXYZ>::Ptr pSceneKeypoints( new pcl::PointCloud<pcl::PointXYZ>() );
	pcl::copyPointCloud( *pScene, sampled_indices.points, *pSceneKeypoints );

	std::cout << "Downsampled cloud." << std::endl;
	std::cout << "Size of keypoint cloud:  " << pSceneKeypoints->size() << std::endl;
	std::cout << "Width of keypoint cloud:  " << pSceneKeypoints->width << std::endl;
	std::cout << "Height of keypoint cloud:  " << pSceneKeypoints->height << std::endl;

	/*
	// --- Compute Descriptors for Keypoints --- //

	std::cout << "Computing keypoint descriptors for scene..." << std::endl;
	pcl::SHOTEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::SHOT352> descrEst;
	//descrEst.setRadiusSearch(mDescrRadius);
	descrEst.setRadiusSearch(0.02f);

	descrEst.setInputCloud( pSceneKeypoints );
	descrEst.setInputNormals( pSceneNormals );
	descrEst.setSearchSurface( pScene );
	pcl::PointCloud<pcl::SHOT352>::Ptr pSceneDescriptors( new pcl::PointCloud<pcl::SHOT352>() );
	descrEst.compute( *pSceneDescriptors );

	std::cout << "Computed keypoint descriptors for scene." << std::endl;
	std::cout << "Size of descriptor cloud:  " << pSceneDescriptors->size() << std::endl;
	std::cout << "Width of descriptor cloud:  " << pSceneDescriptors->width << std::endl;
	std::cout << "Height of descriptor cloud:  " << pSceneDescriptors->height << std::endl;
	*/
}

void recognize(pcl::PointCloud<pcl::PointXYZ>::Ptr scene, pcl::PointCloud<pcl::Normal>::Ptr normals)
{
	// Create recognition object and add the models to find.
	pcl::recognition::ObjRecRANSAC rec(0.3, 10);
	for( std::size_t i = 0; i < models.size(); ++i )
	{
		std::stringstream name;
		name << "cube" << i;
		rec.addModel(*(models[i].first), *(models[i].second), name.str());
	}
	
	std::list<pcl::recognition::ObjRecRANSAC::Output> objs;

	// Try to find objects.
	rec.recognize(*scene, *normals, objs);
	std::cerr << "\tNum objects:  " << objs.size() << std::endl;
}

pcl::PointCloud<pcl::PointNormal> smoothCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud)
{
	// from the resampling tutorial.
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree( new pcl::search::KdTree<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointNormal> mls_points;
	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
	mls.setComputeNormals(true);

	mls.setInputCloud(pCloud);
	mls.setPolynomialFit(true);
	mls.setSearchMethod(tree);
	mls.setSearchRadius(0.03);

	mls.process(mls_points);

	return mls_points;
}

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input)
{
    // Do data processing here...
  
    // run pass through filter to shrink point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_passthrough(new pcl::PointCloud<pcl::PointXYZ>);
    passthrough_z(input, cloud_passthrough);
    passthrough_y(cloud_passthrough);
    passthrough_x(cloud_passthrough);

	processImage(cloud_passthrough);

	smoothCloud(cloud_passthrough);
	
	pcl::PointCloud<pcl::PointXYZ> tempCloud;
	copyPointCloud(*cloud_passthrough, tempCloud);
	visualize(tempCloud.makeShared());
	
	
	// // Visualize and then recognize
	// pcl::PointCloud<pcl::PointXYZ> tempCloud;
	// copyPointCloud(*cloud_passthrough, tempCloud);
	// visualize(tempCloud.makeShared());
	// recognize(cloud_passthrough);
	
    pub.publish(*cloud_passthrough);

    // run ransac to find floor
    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZ>);
    //ransac(cloud_passthrough, cloud_projected);
    //pub.publish(*cloud_projected);
}

void processFile()
{
	// --- Read Scene From File --- //

	pcl::PointCloud<pcl::PointXYZ>::Ptr pScene( new pcl::PointCloud<pcl::PointXYZ>() );

	if (pcl::io::loadPCDFile( "/home/mongeon/medium_block_on_desk_90.pcd", *pScene ) < 0)
	{
		std::cout << "Error loading scene cloud." << std::endl;
		return;
	}

	
	// --- Use Passthrough Filters --- //

	// Cut down on some of the scene clutter.
	std::cerr << "Using passthrough filters." << std::endl;
    passthrough_z(pScene);
    passthrough_y(pScene);
    passthrough_x(pScene);


	// --- Calculate Scene Normals --- //

	std::cerr << "Computing scene normals" << std::endl;
	pSceneNormals.reset( new pcl::PointCloud<pcl::Normal>() );
	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> normEst;
	normEst.setKSearch(10);
	normEst.setInputCloud( pScene );
	normEst.compute( *pSceneNormals );


	// --- Get Rid Of Table --- //

	pcl::PointIndices::Ptr inliers_plane( new pcl::PointIndices );
	pcl::ModelCoefficients::Ptr coefficients_plane( new pcl::ModelCoefficients );

	std::cerr << "Segmenting the table" << std::endl;
	pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg1; 
	seg1.setOptimizeCoefficients( true );
	seg1.setModelType( pcl::SACMODEL_NORMAL_PLANE );
	seg1.setNormalDistanceWeight( 0.05 );
	seg1.setMethodType( pcl::SAC_RANSAC );
	seg1.setMaxIterations( 100 );
	seg1.setDistanceThreshold( 0.03 );
	seg1.setInputCloud( pScene );
	seg1.setInputNormals( pSceneNormals );
	// Obtain the plane inliers and coefficients
	seg1.segment( *inliers_plane, *coefficients_plane );
	//std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;

	// Extract the planar inliers from the input cloud
	std::cerr << "Extracting planar inliers" << std::endl;
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	extract.setInputCloud( pScene );
	extract.setIndices( inliers_plane );
	extract.setNegative( false );

	// Write the planar inliers to disk
	std::cerr << "Filtering out the plane" << std::endl;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane( new pcl::PointCloud<pcl::PointXYZ> );
	extract.filter( *cloud_plane );

	// Remove the planar inliers, extract the rest
	pcl::PointCloud<pcl::PointXYZ>::Ptr filteredScene( new pcl::PointCloud<pcl::PointXYZ> );
	extract.setNegative( true );
	extract.filter( *filteredScene );

	std::cerr << "removing normals" << std::endl;
	pcl::ExtractIndices<pcl::Normal> extract_normals;
	pcl::PointCloud<pcl::Normal>::Ptr filteredSceneNormals( new pcl::PointCloud<pcl::Normal> );
	extract_normals.setNegative( true );
	extract_normals.setInputCloud( pSceneNormals );
	extract_normals.setIndices( inliers_plane );
	extract_normals.filter( *filteredSceneNormals );

	visualize( filteredScene );
	

	// --- Estimate Normals For Scene --- //

	std::cerr << "Computing scene normals" << std::endl;
	pSceneNormals.reset( new pcl::PointCloud<pcl::Normal>() );
	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> normEst2;
	normEst2.setRadiusSearch( 0.01 );
	normEst2.setInputCloud( filteredScene );
	normEst2.compute( *pSceneNormals );


	// --- Estimate Features --- //

	typedef pcl::PointNormal PointNT;
	typedef pcl::PointCloud<PointNT> PointCloudT;
	typedef pcl::FPFHSignature33 FeatureT;
	typedef pcl::FPFHEstimationOMP<PointNT,PointNT,FeatureT> FeatureEstimationT;
	typedef pcl::PointCloud<FeatureT> FeatureCloudT;
	typedef pcl::visualization::PointCloudColorHandlerCustom<PointNT> ColorHandlerT;

	// Estimate features
	
	pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fest;
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr objectFeatures (new pcl::PointCloud<pcl::FPFHSignature33>);
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr sceneFeatures (new pcl::PointCloud<pcl::FPFHSignature33>);
	fest.setRadiusSearch( 0.025 );

	fest.setInputCloud( models[models.size()-1].first );
	fest.setInputNormals( models[models.size()-1].second );
	fest.compute( *objectFeatures );

	fest.setInputCloud( filteredScene );
	fest.setInputNormals( filteredSceneNormals );
	fest.compute( *sceneFeatures );

	// Perform alignment

	// !! If this doesn't work, try doing the downsampling like in the actual tutorial
	
	pcl::SampleConsensusPrerejective<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> align;
	align.setInputSource( models[models.size()-1].first );
	align.setSourceFeatures( objectFeatures );
	align.setInputTarget( filteredScene );
	align.setTargetFeatures( sceneFeatures );
	align.setMaximumIterations( 10000 ); // Number of RANSAC iterations
	align.setNumberOfSamples( 3 ); // Number of points to sample for generating/prerejecting a pose
	align.setCorrespondenceRandomness( 2 ); // Number of nearest features to use
	align.setSimilarityThreshold( 0.9f ); // Polygonal edge length similarity threshold
	float leaf = 0.005f;
	align.setMaxCorrespondenceDistance( 1.5f * leaf ); // Inlier threshold
	align.setInlierFraction( 0.25f ); // Required inlier fraction for accepting a pose hypothesis

	pcl::PointCloud<pcl::PointXYZ>::Ptr objectAligned( new pcl::PointCloud<pcl::PointXYZ> );
	{
		align.align( *objectAligned );
	}

	if( align.hasConverged() )
	{
		std::cerr << "CONVERGED!!" << std::endl;
	}
	else
	{
		std::cerr << "NOTHING" << std::endl;
	}
	

	// --- See What We Have --- //
	
	std::cerr << "Passing smoothed points to visualizer..." << std::endl;
	//visualize( smoothed );
}

int main(int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "pcl_node");
    ros::NodeHandle nh;

	// Create the cube model.
	makeCube(0.051);
	/*
	makeCube(0.1);
	makeCube(0.15);
	makeCube(0.2);
	makeCube(0.25);
	makeCube(0.3);
	*/
	
    // Create a ROS subscriber for the input point cloud
    //ros::Subscriber sub = nh.subscribe ("camera/depth_registered/points", 1, cloud_cb);
	processFile();

    // Create a ROS publisher for the output point cloud
    pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

    // Spin
    ros::spin();
}

// 0. From the beginning make use of PointXYZ instead of PointXYZRGB.
// 1. Remove the table first using planar segmentation - see Ritwik's code


// Backpocket - can use cylinders using Ritwik's code
// Can simulate the output using a Python script and move on to MoveIt!


// youbot_manipulation - can download from canvas, also on the youBot
//  - Jarvis has edited the code to make it work a little better.
//    * changed an rviz file, launch file, 
//  - svenschneider/youbot_manipulation
//    * youbot_arm_kinematics, youbot_arm_moveit
//    * see about using this package
//    * looks like it is done well and is useful
//    * good place to start, especially if I am going to do the 8-DOF control


// Can use topic_tools/throttle to slow down data.



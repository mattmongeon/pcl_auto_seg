#include <limits>
#include <fstream>
#include <vector>
#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/ia_ransac.h>


// --- From pcl_scratch.cpp --- //

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
#include <sstream>


ros::Publisher pub;
pcl::PointCloud<pcl::PointXYZ>::Ptr pCube;
pcl::PointCloud<pcl::Normal>::Ptr pCubeNormals;
pcl::PointCloud<pcl::Normal>::Ptr pSceneNormals;

typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudPtr;
typedef pcl::PointCloud<pcl::Normal>::Ptr NormalCloudPtr;
typedef std::pair<PointCloudPtr, NormalCloudPtr> ModelPair;
std::vector<ModelPair> models;


////////////////////////////////////////////////////////////////////////////////
// From the tutorial "Aligning Object Templates to a Point Cloud"
////////////////////////////////////////////////////////////////////////////////


void visualize(std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr > clouds)
{
    //init visualizer
	pcl::visualization::PCLVisualizer::Ptr visualizer_o_Ptr (new pcl::visualization::PCLVisualizer());
	visualizer_o_Ptr->setSize(640, 480);
	visualizer_o_Ptr->setPosition(640, 0);
	visualizer_o_Ptr->setBackgroundColor(0x00, 0x00, 0x00);
	visualizer_o_Ptr->addCoordinateSystem(1.0);
	visualizer_o_Ptr->initCameraParameters();

	for( std::size_t i = 0; i < clouds.size(); ++i )
	{
		std::stringstream name;
		name << "cloud" << i;
		visualizer_o_Ptr->addPointCloud(clouds[i], name.str());
	}

	for( std::size_t i = 0; i < models.size(); ++i )
	{
		std::stringstream name;
		name << "cube" << i;
		visualizer_o_Ptr->addPointCloud(models[i].first, name.str());
	}
	
	//reload visualizer content
	visualizer_o_Ptr->spinOnce(10000000);
}



class FeatureCloud
{
public:
    // A bit of shorthand
    typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
    typedef pcl::PointCloud<pcl::Normal> SurfaceNormals;
    typedef pcl::PointCloud<pcl::FPFHSignature33> LocalFeatures;
    typedef pcl::search::KdTree<pcl::PointXYZ> SearchMethod;

    FeatureCloud () :
		search_method_xyz_ (new SearchMethod),
		normal_radius_ (0.02f),
		feature_radius_ (0.02f)
		{}

    ~FeatureCloud () {}

    // Process the given cloud
    void setInputCloud (PointCloud::Ptr xyz)
		{
			xyz_ = xyz;
			processInput ();
		}

    // Load and process the cloud in the given PCD file
    void
    loadInputCloud (const std::string &pcd_file)
		{
			xyz_ = PointCloud::Ptr (new PointCloud);
			pcl::io::loadPCDFile (pcd_file, *xyz_);
			processInput ();
		}

	void make90Cube(float side_m)
	{
		// --- Define Constants --- //

		float cubeShortSide = side_m;  // Horizontal side length (m)
		float cubeLongSide = side_m;   // vertical side length (m)

		// How many points on a long side (esitmate) for making test cubes, either
		// for ICP registration or for testing fit_planes.cpp
		const int nLongSideHighRes = 15;
		const int nShortSideHighRes = 15;

	
		// --- Generate Cube Model Point Cloud --- //

		int npointsBottomFace = nShortSideHighRes * nShortSideHighRes;
		int npointsSideFace = nShortSideHighRes * nLongSideHighRes;
		int npointsTotal = npointsBottomFace + 2*npointsSideFace;
	
		float dxShortSide = cubeShortSide / (float)nShortSideHighRes;
		float dxLongSide =  cubeLongSide /  (float)nLongSideHighRes;
	
		pcl::PointCloud<pcl::PointXYZ>::Ptr model( new pcl::PointCloud<pcl::PointXYZ>() );
		model->width = npointsTotal;
		model->height = 1;
	
		// make the top and bottom cap faces
		// these go at y = +- cubeLongSide /2 
		// from x,z = -ls/2, -ls/2  to x,z = ls/2, ls/2
		int counter = 0;
		float xval, yval, zval;
		float xOffset, yOffset, zOffset;
	
		// bottom face
		yval = -cubeLongSide / 2;
		xOffset = -cubeShortSide / 2;
		zOffset = -cubeShortSide / 2;
		for(int i = 0; i < nShortSideHighRes; i++){
			for(int j = 0; j < nShortSideHighRes; j++){
				model->points.push_back(pcl::PointXYZ());
				model->points[counter].x = i*dxShortSide + xOffset;
				model->points[counter].y = yval;
				model->points[counter].z = j*dxShortSide + zOffset;
				counter++;
			}
		}


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

		xval = cubeShortSide / 2;
		zOffset = -cubeShortSide / 2;
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


		// --- Add It To The Library --- //

		setInputCloud(model);
	}

	void makeAlignedCube(float side_m)
	{
		// --- Define Constants --- //

		float cubeShortSide = side_m;  // Horizontal side length (m)
		float cubeLongSide = side_m;   // vertical side length (m)

		// How many points on a long side (esitmate) for making test cubes, either
		// for ICP registration or for testing fit_planes.cpp
		const int nLongSideHighRes = 15;
		const int nShortSideHighRes = 15;

	
		// --- Generate Cube Model Point Cloud --- //

		int npointsBottomFace = nShortSideHighRes * nShortSideHighRes;
		int npointsSideFace = nShortSideHighRes * nLongSideHighRes;
		int npointsTotal = npointsBottomFace + npointsSideFace;
	
		float dxShortSide = cubeShortSide / (float)nShortSideHighRes;
		float dxLongSide =  cubeLongSide /  (float)nLongSideHighRes;
	
		pcl::PointCloud<pcl::PointXYZ>::Ptr model( new pcl::PointCloud<pcl::PointXYZ>() );
		model->width = npointsTotal;
		model->height = 1;
	
		// make the top and bottom cap faces
		// these go at y = +- cubeLongSide /2 
		// from x,z = -ls/2, -ls/2  to x,z = ls/2, ls/2
		int counter = 0;
		float xval, yval, zval;
		float xOffset, yOffset, zOffset;
	
		yval = -cubeLongSide / 2;
		xOffset = -cubeShortSide / 2;
		zOffset = -cubeShortSide / 2;
		for(int i = 0; i < nShortSideHighRes; i++){
			for(int j = 0; j < nShortSideHighRes; j++){
				model->points.push_back(pcl::PointXYZ());
				model->points[counter].x = i*dxShortSide + xOffset;
				model->points[counter].y = yval;
				model->points[counter].z = j*dxShortSide + zOffset;
				counter++;
			}
		}

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

		// --- Add It To The Library --- //

		setInputCloud(model);
	}

    // Get a pointer to the cloud 3D points
    PointCloud::Ptr
    getPointCloud () const
		{
			return (xyz_);
		}

    // Get a pointer to the cloud of 3D surface normals
    SurfaceNormals::Ptr
    getSurfaceNormals () const
	{
		return (normals_);
	}

    // Get a pointer to the cloud of feature descriptors
    LocalFeatures::Ptr
    getLocalFeatures () const
	{
		return (features_);
	}

protected:
    // Compute the surface normals and local features
    void processInput()
	{
		computeSurfaceNormals();
		computeLocalFeatures();
	}

    // Compute the surface normals
    void computeSurfaceNormals ()
	{
		std::cerr << "ComputeSurfaceNormals()" << std::endl;
		normals_ = SurfaceNormals::Ptr (new SurfaceNormals);

		pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> norm_est;
		norm_est.setInputCloud (xyz_);
		norm_est.setSearchMethod (search_method_xyz_);
		norm_est.setRadiusSearch (normal_radius_);
		norm_est.compute (*normals_);
	}

    // Compute the local feature descriptors
    void
    computeLocalFeatures ()
		{
			std::cerr << "ComputeLocalFeatures()" << std::endl;
			features_ = LocalFeatures::Ptr (new LocalFeatures);

			pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_est;
			fpfh_est.setInputCloud (xyz_);
			fpfh_est.setInputNormals (normals_);
			fpfh_est.setSearchMethod (search_method_xyz_);
			fpfh_est.setRadiusSearch (feature_radius_);
			fpfh_est.compute (*features_);
		}

private:
    // Point cloud data
    PointCloud::Ptr xyz_;
    SurfaceNormals::Ptr normals_;
    LocalFeatures::Ptr features_;
    SearchMethod::Ptr search_method_xyz_;

    // Parameters
    float normal_radius_;
    float feature_radius_;
};

class TemplateAlignment
{
public:

    // A struct for storing alignment results
    struct Result
    {
		float fitness_score;
		Eigen::Matrix4f final_transformation;
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    TemplateAlignment () :
		min_sample_distance_ (0.05f),
		max_correspondence_distance_ (0.01f*0.01f),
		nr_iterations_ (500)
		{
			// Intialize the parameters in the Sample Consensus Intial Alignment (SAC-IA) algorithm
			sac_ia_.setMinSampleDistance (min_sample_distance_);
			sac_ia_.setMaxCorrespondenceDistance (max_correspondence_distance_);
			sac_ia_.setMaximumIterations (nr_iterations_);
		}

    ~TemplateAlignment () {}

    // Set the given cloud as the target to which the templates will be aligned
    void
    setTargetCloud (FeatureCloud &target_cloud)
		{
			target_ = target_cloud;
			sac_ia_.setInputTarget (target_cloud.getPointCloud ());
			sac_ia_.setTargetFeatures (target_cloud.getLocalFeatures ());
		}

    // Add the given cloud to the list of template clouds
    void
    addTemplateCloud (FeatureCloud &template_cloud)
		{
			templates_.push_back (template_cloud);
		}

    // Align the given template cloud to the target specified by setTargetCloud ()
    void
    align (FeatureCloud &template_cloud, TemplateAlignment::Result &result)
		{
			sac_ia_.setInputCloud (template_cloud.getPointCloud ());
			sac_ia_.setSourceFeatures (template_cloud.getLocalFeatures ());

			pcl::PointCloud<pcl::PointXYZ> registration_output;
			sac_ia_.align (registration_output);

			result.fitness_score = (float) sac_ia_.getFitnessScore (max_correspondence_distance_);
			result.final_transformation = sac_ia_.getFinalTransformation ();
		}

    // Align all of template clouds set by addTemplateCloud to the target specified by setTargetCloud ()
    void
    alignAll (std::vector<TemplateAlignment::Result, Eigen::aligned_allocator<Result> > &results)
		{
			results.resize (templates_.size ());
			for (size_t i = 0; i < templates_.size (); ++i)
			{
				align (templates_[i], results[i]);
			}
		}

    // Align all of template clouds to the target cloud to find the one with best alignment score
    int
    findBestAlignment (TemplateAlignment::Result &result)
		{
			// Align all of the templates to the target cloud
			std::vector<Result, Eigen::aligned_allocator<Result> > results;
			alignAll (results);

			// Find the template with the best (lowest) fitness score
			float lowest_score = std::numeric_limits<float>::infinity ();
			int best_template = 0;
			for (size_t i = 0; i < results.size (); ++i)
			{
				const Result &r = results[i];
				if (r.fitness_score < lowest_score)
				{
					lowest_score = r.fitness_score;
					best_template = (int) i;
				}
			}

			// Output the best alignment
			result = results[best_template];
			return (best_template);
		}

private:
    // A list of template clouds and the target to which they will be aligned
    std::vector<FeatureCloud> templates_;
    FeatureCloud target_;

    // The Sample Consensus Initial Alignment (SAC-IA) registration routine and its parameters
    pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sac_ia_;
    float min_sample_distance_;
    float max_correspondence_distance_;
    int nr_iterations_;
};


// Align a collection of object templates to a sample point cloud
void processFile()
{
	// Load the object templates specified in the object_templates.txt file
	std::vector<FeatureCloud> object_templates;
	object_templates.clear();

	FeatureCloud template_cloud1;
	std::cerr << "Cube size 0.025" << std::endl;
	template_cloud1.make90Cube(0.025);
	object_templates.push_back(template_cloud1);

	FeatureCloud template_cloud2;
	std::cerr << "Cube size 0.04" << std::endl;
	template_cloud2.make90Cube(0.04);
	object_templates.push_back(template_cloud2);
	
	FeatureCloud template_cloud3;
	std::cerr << "Cube size 0.051" << std::endl;
	template_cloud3.make90Cube(0.051);
	object_templates.push_back(template_cloud3);

	FeatureCloud template_cloud4;
	std::cerr << "Cube size 0.025" << std::endl;
	template_cloud4.makeAlignedCube(0.025);
	object_templates.push_back(template_cloud4);

	FeatureCloud template_cloud5;
	std::cerr << "Cube size 0.04" << std::endl;
	template_cloud5.makeAlignedCube(0.04);
	object_templates.push_back(template_cloud5);
	
	FeatureCloud template_cloud6;
	std::cerr << "Cube size 0.051" << std::endl;
	template_cloud6.makeAlignedCube(0.051);
	object_templates.push_back(template_cloud6);

	/*
	FeatureCloud template_cloud4;
	std::cerr << "Cube size 0.075" << std::endl;
	template_cloud4.makeCube(0.075);
	object_templates.push_back(template_cloud4);

	FeatureCloud template_cloud5;
	std::cerr << "Cube size 0.15" << std::endl;
	template_cloud5.makeCube(0.15);
	object_templates.push_back(template_cloud5);
	
	FeatureCloud template_cloud6;
	std::cerr << "Cube size 0.2" << std::endl;
	template_cloud6.makeCube(0.2);
	object_templates.push_back(template_cloud6);

	FeatureCloud template_cloud7;
	std::cerr << "Cube size 0.25" << std::endl;
	template_cloud7.makeCube(0.25);
	object_templates.push_back(template_cloud7);

	FeatureCloud template_cloud8;
	std::cerr << "Cube size 0.3" << std::endl;
	template_cloud8.makeCube(0.3);
	object_templates.push_back(template_cloud8);
	*/

	//--- Load the target cloud PCD file --- //
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile( "/home/mongeon/medium_block_on_desk_align.pcd", *cloud );


	// --- Z-Filter And Downsample Cloud --- //
	
	std::cerr << "Preprocessing cloud" << std::endl;
	// Preprocess the cloud by...
	// ...removing distant points
	const float depth_limit = 1.5;
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud (cloud);
	pass.setFilterFieldName ("z");
	pass.setFilterLimits (0, 1.0);
	pass.filter (*cloud);


	/*
	// --- Downsample Cloud --- //

	// ... and downsampling the point cloud
	const float voxel_grid_size = 0.005f;
	pcl::VoxelGrid<pcl::PointXYZ> vox_grid;
	vox_grid.setInputCloud (cloud);
	vox_grid.setLeafSize (voxel_grid_size, voxel_grid_size, voxel_grid_size);
	//vox_grid.filter (*cloud); // Please see this http://www.pcl-developers.org/Possible-problem-in-new-VoxelGrid-implementation-from-PCL-1-5-0-td5490361.html
	pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud (new pcl::PointCloud<pcl::PointXYZ>); 
	vox_grid.filter (*tempCloud);
	cloud = tempCloud;
	*/
	
	// --- Calculate Scene Normals --- //

	std::cerr << "Computing scene normals" << std::endl;
	pcl::PointCloud<pcl::Normal>::Ptr pSceneNormals( new pcl::PointCloud<pcl::Normal>() );
	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> normEst;
	normEst.setKSearch(10);
	normEst.setInputCloud( cloud );
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
	seg1.setDistanceThreshold( 0.075 );
	seg1.setInputCloud( cloud );
	seg1.setInputNormals( pSceneNormals );
	// Obtain the plane inliers and coefficients
	seg1.segment( *inliers_plane, *coefficients_plane );
	//std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;

	// Extract the planar inliers from the input cloud
	std::cerr << "Extracting planar inliers" << std::endl;
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	extract.setInputCloud( cloud );
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
	
	/*
	// --- Smooth Surfaces --- //

	// Borrowed from tutorial "Smoothing and normal estimation based on polynomial reconstruction"
	std::cerr << "Smoothing surfaces" << std::endl;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree( new pcl::search::KdTree<pcl::PointXYZ>() );
	pcl::PointCloud<pcl::PointNormal> mls_points;
	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
	mls.setComputeNormals(true);
	mls.setInputCloud(filteredScene);
	mls.setPolynomialFit(true);
	mls.setSearchMethod(tree);
	mls.setSearchRadius(0.03);
	mls.process(mls_points);
	pcl::PointCloud<pcl::PointXYZ>::Ptr pts( new pcl::PointCloud<pcl::PointXYZ>() );
	copyPointCloud(mls_points, *pts);
	filteredScene = pts;
	*/

	// --- Set Our Target Cloud --- //
	
	// Assign to the target FeatureCloud
	FeatureCloud target_cloud;
	std::cerr << "Setting intput cloud for target FeatureCloud" << std::endl;
	target_cloud.setInputCloud(filteredScene);


	// --- Set Up Template Container --- //
	
	TemplateAlignment template_align;
	std::cerr << "Entering loop for adding template cloud" << std::endl;
	for (size_t i = 0; i < object_templates.size (); ++i)
	{
		std::cerr << "\tAdding cloud " << i << std::endl;
		template_align.addTemplateCloud (object_templates[i]);
	}
	std::cerr << "Setting target cloud for the TemplateAlignment object" << std::endl;
	template_align.setTargetCloud( target_cloud );


	// --- Align Templates --- //
	
	// Find the best template alignment
	TemplateAlignment::Result best_alignment;
	std::cerr << "Finding best alignment" << std::endl;
	int best_index = template_align.findBestAlignment (best_alignment);
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
	std::cerr << "t = < " << translation(0) << ", " << translation(1) << ", " << translation(2) << " >" << std::endl;


	// --- Visualize --- //
	
	// Save the aligned template for visualization
	pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
	pcl::transformPointCloud (*best_template.getPointCloud (), transformed_cloud, best_alignment.final_transformation);

	std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr > clouds;
	clouds.push_back(transformed_cloud.makeShared());
	clouds.push_back(filteredScene);
	visualize(clouds);
}


int main(int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "pcl_node");
    ros::NodeHandle nh;
	
    // Create a ROS subscriber for the input point cloud
    //ros::Subscriber sub = nh.subscribe ("camera/depth_registered/points", 1, cloud_cb);
	processFile();

    // Create a ROS publisher for the output point cloud
    pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

    // Spin
    ros::spin();
}

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
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>

#include <iostream>

class cMatcher
{
public:

	cMatcher(ros::NodeHandle nh)
		: mpCube( new Cloud() ),
		  mpCubeNormals( new pcl::PointCloud<pcl::Normal>() ),
		  mpCubeKeypoints( new pcl::PointCloud<pcl::PointXYZ>() ),
		  mpCubeDescriptors( new pcl::PointCloud<DescriptorType>() ),
		  mUniformSampling(),
		  mNormEst(),
		  mSub(),
		  mDescrRadius( 0.02f ),
		  mUseHough( true ),
		  mRF_rad( 0.015f ),
		  mCG_size( 0.01f ),
		  mCG_thresh( 5.0f )
	{
		mPub = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("pcl_file", 100);
		
		std::cout << "Started construction." << std::endl;

		// --- Define Constants --- //

		const float cubeShortSide = 0.045;  // Horizontal side length (m)
		const float cubeLongSide = 0.045;   // vertical side length (m)

		// How many points on a long side (esitmate) for making test cubes, either
		// for ICP registration or for testing fit_planes.cpp
		const int nLongSideHighRes = 54;
		const int nLongSideLowRes = 27;

		const int nShortSideHighRes = 18;
		const int nShortSideLowRes = 9;
		
		// --- Generate Cube Model Point Cloud --- //

		int npointsBottomFace = nShortSideLowRes* nShortSideLowRes;
		int npointsSideFace = nShortSideLowRes* nLongSideLowRes;
		int npointsTotal = 2*npointsBottomFace + 4 *npointsSideFace;
	
		float dxShortSide = cubeShortSide / (float)nShortSideLowRes;
		float dxLongSide =  cubeLongSide /  (float)nLongSideLowRes;
	
		std::cerr << "# dxShortSide: " << dxShortSide << std::endl;
		std::cerr << "# dxLongSide: " << dxLongSide << std::endl;
		std::cerr << "# npointsTotal: " << npointsTotal << std::endl;
	
		mpCube->width = npointsTotal;
		mpCube->height = 1;
		mpCube->points.resize(npointsTotal); // allocate space for all the points we need
	
		// make the top and bottom cap faces
		// these go at y = +- cubeLongSide /2 
		// from x,z = -ls/2, -ls/2  to x,z = ls/2, ls/2
		int counter = 0;
		float xval, yval, zval;
		float xOffset, yOffset, zOffset;
	
		// top face
		yval = cubeLongSide / 2;
		xOffset = - cubeShortSide / 2;
		zOffset = - cubeShortSide / 2;
		for(int i = 0; i < nShortSideLowRes; i++){
			for(int j = 0; j < nShortSideLowRes; j++){
				mpCube->points[counter].x = i*dxShortSide +  xOffset;
				mpCube->points[counter].y = yval;
				mpCube->points[counter].z = j*dxShortSide + zOffset;
				counter++;
			}
		}

		// bottom face
		yval = -cubeLongSide / 2;
		xOffset = - cubeShortSide / 2;
		zOffset = - cubeShortSide / 2;
		for(int i = 0; i < nShortSideLowRes; i++){
			for(int j = 0; j < nShortSideLowRes; j++){
				mpCube->points[counter].x = i*dxShortSide + xOffset;
				mpCube->points[counter].y = yval;
				mpCube->points[counter].z = j*dxShortSide + zOffset;
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
		for(int i = 0; i < nShortSideLowRes; i++){
			for(int j = 0; j < nLongSideLowRes; j++){
				mpCube->points[counter].x = i*dxShortSide + xOffset;
				mpCube->points[counter].y = j*dxLongSide + yOffset;
				mpCube->points[counter].z = zval;
				counter++;
			}
		}

		zval = cubeShortSide / 2;
		xOffset = - cubeShortSide / 2;
		yOffset = - cubeLongSide / 2;
		for(int i = 0; i < nShortSideLowRes; i++){
			for(int j = 0; j < nLongSideLowRes; j++){
				mpCube->points[counter].x = i*dxShortSide + xOffset;
				mpCube->points[counter].y = j*dxLongSide +  yOffset;
				mpCube->points[counter].z = zval;
				counter++;
			}
		}

		xval = -cubeShortSide / 2;
		zOffset = - cubeShortSide / 2;
		yOffset = - cubeLongSide / 2;
		for(int i = 0; i < nShortSideLowRes; i++){
			for(int j = 0; j < nLongSideLowRes; j++){
				mpCube->points[counter].x = xval;
				mpCube->points[counter].y = j*dxLongSide + yOffset;
				mpCube->points[counter].z = i*dxShortSide + zOffset;
				counter++;
			}
		}

		xval = cubeShortSide / 2;
		zOffset = - cubeShortSide / 2;
		yOffset = - cubeLongSide / 2;
		for(int i = 0; i < nShortSideLowRes; i++){
			for(int j = 0; j < nLongSideLowRes; j++){
				mpCube->points[counter].x = xval;
				mpCube->points[counter].y = j*dxLongSide + yOffset;
				mpCube->points[counter].z = i*dxShortSide + zOffset;
				counter++;
			}
		}
		
		std::cout << "Generated cube model point cloud." << std::endl;
		

		// --- Compute Normals --- //

		mNormEst.setKSearch(10);
		mNormEst.setInputCloud( mpCube );
		mNormEst.compute( *mpCubeNormals );

		std::cout << "Computed model normals." << std::endl;


		// --- Downsample Model Cloud --- //

		float model_ss_ = 0.005f;

		pcl::PointCloud<int> sampled_indices;
				
		mUniformSampling.setInputCloud( mpCube );
		mUniformSampling.setRadiusSearch( model_ss_ );
		mUniformSampling.compute( sampled_indices );
		pcl::copyPointCloud( *mpCube, sampled_indices.points, *mpCubeKeypoints );

		std::cout << "Downsampled cloud model." << std::endl;


		// --- Compute Descriptors for Keypoints --- //

		pcl::SHOTEstimationOMP<pcl::PointXYZ, pcl::Normal, DescriptorType> descrEst;
		descrEst.setRadiusSearch(mDescrRadius);

		descrEst.setInputCloud( mpCubeKeypoints );
		descrEst.setInputNormals( mpCubeNormals );
		descrEst.setSearchSurface( mpCube );
		descrEst.compute( *mpCubeDescriptors );

		std::cout << "Computed keypoint descriptors for model." << std::endl;

std::cout << "Size of cube descriptors cloud:  " << mpCubeDescriptors->size() << std::endl;
std::cout << "Width of cube descriptors cloud:  " << mpCubeDescriptors->width << std::endl;
std::cout << "Height of cube descriptors cloud:  " << mpCubeDescriptors->height << std::endl;

		std::cout << "End constructor." << std::endl << std::endl;
	}

	~cMatcher()
	{
	}

	void PublishSubscribe(ros::NodeHandle& nh)
	{
	    // Subscriber for taking in the input point cloud.
		//mSub = nh.subscribe("camera/depth_registered/points", 1, &cMatcher::CloudCallback, this);
	}

		//void CloudCallback(const sensor_msgs::PointCloud2ConstPtr& input)
	void ProcessFile()
	{
        // --- Read Scene From File --- //

		pcl::PointCloud<pcl::PointXYZ>::Ptr pScene( new pcl::PointCloud<pcl::PointXYZ>() );

        if (pcl::io::loadPCDFile( "/home/mongeon/medium_block_on_desk_90.pcd", *pScene ) < 0)
		{
            std::cout << "Error loading scene cloud." << std::endl;
            return;
        }

		std::cout << "Scene loaded." << std::endl;

		
        // --- Read From ROS --- //

		// // Read in the point cloud from the ROS message.
		// pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud(new pcl::PointCloud<pcl::PointXYZ>);
		// pcl::fromROSMsg(*input, *pCloud);


		// --- Compute Normals --- //
		
		pcl::PointCloud<pcl::Normal>::Ptr pSceneNormals( new pcl::PointCloud<pcl::Normal>() );
		mNormEst.setInputCloud( pScene );
		mNormEst.compute( *pSceneNormals );

		std::cout << "Scene normals computed." << std::endl;
std::cout << "Size of normal cloud:  " << pSceneNormals->size() << std::endl;
std::cout << "Width of normal cloud:  " << pSceneNormals->width << std::endl;
std::cout << "Height of normal cloud:  " << pSceneNormals->height << std::endl;


		// --- Downsample Scene Cloud --- //

		pcl::PointCloud<int> sampled_indices;

		float scene_ss_ = 0.01f;
		
		mUniformSampling.setInputCloud( pScene );
		mUniformSampling.setRadiusSearch( scene_ss_ );
		mUniformSampling.compute( sampled_indices );
		pcl::PointCloud<pcl::PointXYZ>::Ptr pSceneKeypoints( new pcl::PointCloud<pcl::PointXYZ>() );
		pcl::copyPointCloud( *pScene, sampled_indices.points, *pSceneKeypoints );

		std::cout << "Downsampled cloud." << std::endl;
std::cout << "Size of keypoint cloud:  " << pSceneKeypoints->size() << std::endl;
std::cout << "Width of keypoint cloud:  " << pSceneKeypoints->width << std::endl;
std::cout << "Height of keypoint cloud:  " << pSceneKeypoints->height << std::endl;


		// --- Compute Descriptors for Keypoints --- //

std::cout << "Computing keypoint descriptors for scene..." << std::endl;
		pcl::SHOTEstimationOMP<pcl::PointXYZ, pcl::Normal, DescriptorType> descrEst;
		//descrEst.setRadiusSearch(mDescrRadius);
		descrEst.setRadiusSearch(0.15f);

		descrEst.setInputCloud( pSceneKeypoints );
		descrEst.setInputNormals( pSceneNormals );
		descrEst.setSearchSurface( pScene );
		pcl::PointCloud<DescriptorType>::Ptr pSceneDescriptors( new pcl::PointCloud<DescriptorType>() );
		descrEst.compute( *pSceneDescriptors );

		std::cout << "Computed keypoint descriptors for scene." << std::endl;
std::cout << "Size of descriptor cloud:  " << pSceneDescriptors->size() << std::endl;
std::cout << "Width of descriptor cloud:  " << pSceneDescriptors->width << std::endl;
std::cout << "Height of descriptor cloud:  " << pSceneDescriptors->height << std::endl;


		// --- Find Model-Scene Correspondences with KDTree --- //

std::cout << "Finding model-scene correspondences..." << std::endl;

		pcl::CorrespondencesPtr modelSceneCorrs( new pcl::Correspondences() );

		pcl::KdTreeFLANN<DescriptorType> match_search;
		match_search.setInputCloud( mpCubeDescriptors );

		//  For each scene keypoint descriptor, find nearest neighbor into the model keypoints descriptor cloud and add it to the correspondences vector.
		for (size_t i = 0; i < pSceneDescriptors->size (); ++i)
		{
			std::vector<int> neigh_indices (1);
			std::vector<float> neigh_sqr_dists (1);
			if( !pcl_isfinite(pSceneDescriptors->at (i).descriptor[0]) ) //skipping NaNs
			{
				continue;
			}
			
			int found_neighs = match_search.nearestKSearch( pSceneDescriptors->at (i), 1, neigh_indices, neigh_sqr_dists );
			if(found_neighs == 1 && neigh_sqr_dists[0] < 0.25f) //  add match only if the squared descriptor distance is less than 0.25 (SHOT descriptor distances are between 0 and 1 by design)
			{
				pcl::Correspondence corr( neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0] );
				modelSceneCorrs->push_back( corr );
			}
		}
		
		std::cout << "Correspondences found: " << modelSceneCorrs->size () << std::endl;


		// --- Actual Clustering --- //

		std::cout << "Clustering..." << std::endl;
		std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
		std::vector<pcl::Correspondences> clustered_corrs;

		//  Using Hough3D
		if( mUseHough )
		{
			std::cout << "Using Hough..." << std::endl;
			//
			//  Compute (Keypoints) Reference Frames only for Hough
			//
			pcl::PointCloud<RFType>::Ptr model_rf (new pcl::PointCloud<RFType> ());
			pcl::PointCloud<RFType>::Ptr scene_rf (new pcl::PointCloud<RFType> ());

			pcl::BOARDLocalReferenceFrameEstimation<pcl::PointXYZ, pcl::Normal, RFType> rf_est;
			rf_est.setFindHoles(true);
			rf_est.setRadiusSearch(mRF_rad);

			rf_est.setInputCloud( mpCubeKeypoints );
			rf_est.setInputNormals( mpCubeNormals );
			rf_est.setSearchSurface( mpCube );
			rf_est.compute( *model_rf );

			rf_est.setInputCloud( pSceneKeypoints );
			rf_est.setInputNormals( pSceneNormals );
			rf_est.setSearchSurface( pScene );
			rf_est.compute( *scene_rf );

			//  Clustering
			pcl::Hough3DGrouping<pcl::PointXYZ, pcl::PointXYZ, RFType, RFType> clusterer;
			clusterer.setHoughBinSize( mCG_size );
			clusterer.setHoughThreshold( mCG_thresh );
			clusterer.setUseInterpolation( true );
			clusterer.setUseDistanceWeight( false );

			clusterer.setInputCloud( mpCubeKeypoints);
			clusterer.setInputRf( model_rf );
			clusterer.setSceneCloud( pSceneKeypoints );
			clusterer.setSceneRf( scene_rf );
			clusterer.setModelSceneCorrespondences( modelSceneCorrs );

			//clusterer.cluster (clustered_corrs);
			clusterer.recognize( rototranslations, clustered_corrs );
		}
		else // Using GeometricConsistency
		{
			std::cout << "Using Geometric Consistency..." << std::endl;
			pcl::GeometricConsistencyGrouping<pcl::PointXYZ, pcl::PointXYZ> gc_clusterer;
			gc_clusterer.setGCSize( mCG_size );
			gc_clusterer.setGCThreshold( mCG_thresh );

			gc_clusterer.setInputCloud( mpCubeKeypoints );
			gc_clusterer.setSceneCloud( pSceneKeypoints );
			gc_clusterer.setModelSceneCorrespondences( modelSceneCorrs );

			//gc_clusterer.cluster (clustered_corrs);
			gc_clusterer.recognize (rototranslations, clustered_corrs);
		}


		// --- Output Results --- //

		std::cout << "Model instances found: " << rototranslations.size () << std::endl;
		for (size_t i = 0; i < rototranslations.size (); ++i)
		{
			std::cout << "\n    Instance " << i + 1 << ":" << std::endl;
			std::cout << "        Correspondences belonging to this instance: " << clustered_corrs[i].size () << std::endl;

			// Print the rotation matrix and translation vector
			Eigen::Matrix3f rotation = rototranslations[i].block<3,3>(0, 0);
			Eigen::Vector3f translation = rototranslations[i].block<3,1>(0, 3);

			printf ("\n");
			printf ("            | %6.3f %6.3f %6.3f | \n", rotation (0,0), rotation (0,1), rotation (0,2));
			printf ("        R = | %6.3f %6.3f %6.3f | \n", rotation (1,0), rotation (1,1), rotation (1,2));
			printf ("            | %6.3f %6.3f %6.3f | \n", rotation (2,0), rotation (2,1), rotation (2,2));
			printf ("\n");
			printf ("        t = < %0.3f, %0.3f, %0.3f >\n", translation (0), translation (1), translation (2));
		}
	}

	
private:

	typedef pcl::PointCloud<pcl::PointXYZ> Cloud;
	typedef Cloud::Ptr CloudPtr;
	typedef pcl::SHOT352 DescriptorType;
	typedef pcl::ReferenceFrame RFType;

	// --- Data Members --- //

	CloudPtr mpCube;
	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> mNormEst;
	pcl::PointCloud<pcl::Normal>::Ptr mpCubeNormals;
	pcl::PointCloud<pcl::PointXYZ>::Ptr mpCubeKeypoints;
	pcl::UniformSampling<pcl::PointXYZ> mUniformSampling;
	pcl::PointCloud<DescriptorType>::Ptr mpCubeDescriptors;
	
	ros::Subscriber mSub;

	ros::Publisher mPub;

	float mDescrRadius;
	bool mUseHough;
	float mRF_rad;
	float mCG_size;
	float mCG_thresh;
};


////////////////////////////////////////////////////////////////////////////////
//  MAIN
////////////////////////////////////////////////////////////////////////////////


int main(int argc, char** argv)
{
    // Initialize ROS
    ros::init(argc, argv, "pcl_auto_seg");
    ros::NodeHandle nh;

	cMatcher m(nh);
	//m.PublishSubscribe(nh);
	m.ProcessFile();

    ros::spin();

    return 0;
}


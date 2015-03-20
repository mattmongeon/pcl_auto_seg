#include "pcl_auto_seg/feature_cloud.h"
#include <pcl/io/pcd_io.h>


FeatureCloud::FeatureCloud()
	:	search_method_xyz_ (new SearchMethod),
		normal_radius_ (0.02f),
		feature_radius_ (0.02f)
{

}

FeatureCloud::~FeatureCloud()
{

}

void FeatureCloud::setInputCloud( PointCloud::Ptr xyz )
{
	xyz_ = xyz;
	processInput();
}

void FeatureCloud::loadInputCloud(const std::string &pcd_file)
{
	xyz_ = PointCloud::Ptr(new PointCloud);
	pcl::io::loadPCDFile(pcd_file, *xyz_);
	processInput();
}

void FeatureCloud::makeCube(float side_m)
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

FeatureCloud::PointCloud::Ptr FeatureCloud::getPointCloud() const
{
	return xyz_;
}

FeatureCloud::SurfaceNormals::Ptr FeatureCloud::getSurfaceNormals() const
{
	return normals_;
}

FeatureCloud::LocalFeatures::Ptr FeatureCloud::getLocalFeatures() const
{
	return features_;
}

void FeatureCloud::processInput()
{
	computeSurfaceNormals();
	computeLocalFeatures();
}

void FeatureCloud::computeSurfaceNormals()
{
	normals_ = SurfaceNormals::Ptr (new SurfaceNormals);
	
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> norm_est;
	norm_est.setInputCloud (xyz_);
	norm_est.setSearchMethod (search_method_xyz_);
	norm_est.setRadiusSearch (normal_radius_);
	norm_est.compute (*normals_);
}

void FeatureCloud::computeLocalFeatures()
{
	features_ = LocalFeatures::Ptr( new LocalFeatures );

	pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_est;
	fpfh_est.setInputCloud( xyz_ );
	fpfh_est.setInputNormals( normals_ );
	fpfh_est.setSearchMethod( search_method_xyz_ );
	fpfh_est.setRadiusSearch( feature_radius_ );
	fpfh_est.compute( *features_ );
}


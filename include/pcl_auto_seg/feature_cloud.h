#ifndef INCLUDED_FEATURE_CLOUD_H
#define INCLUDED_FEATURE_CLOUD_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh.h>


class FeatureCloud
{
public:
    // A bit of shorthand
    typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
    typedef pcl::PointCloud<pcl::Normal> SurfaceNormals;
    typedef pcl::PointCloud<pcl::FPFHSignature33> LocalFeatures;
    typedef pcl::search::KdTree<pcl::PointXYZ> SearchMethod;

    FeatureCloud();

    ~FeatureCloud();

    // Process the given cloud
    void setInputCloud( PointCloud::Ptr xyz );

    // Load and process the cloud in the given PCD file
    void loadInputCloud( const std::string &pcd_file );

	void makeCube(float side_m);

    // Get a pointer to the cloud 3D points
    PointCloud::Ptr getPointCloud() const;

    // Get a pointer to the cloud of 3D surface normals
    SurfaceNormals::Ptr getSurfaceNormals() const;

    // Get a pointer to the cloud of feature descriptors
    LocalFeatures::Ptr getLocalFeatures() const;

protected:
    // Compute the surface normals and local features
    void processInput();

    // Compute the surface normals
    void computeSurfaceNormals();

    // Compute the local feature descriptors
	void computeLocalFeatures();

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




#endif // INCLUDED_FEATURE_CLOUD_H

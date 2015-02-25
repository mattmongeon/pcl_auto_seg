#ifndef SALIENTREGION_H_INCLUDED
#define SALIENTREGION_H_INCLUDED

#include <pcl/point_types.h>
#include <vector>
#include <utility>

// A 3D cluster representing a salient region in the image.  This class is part
// of the 
class cSalientRegion
{
public:

    //------------------------------------------------------------------------//
    //----------------------------  CONSTRUCTION  ----------------------------//
    //------------------------------------------------------------------------//

    /**
     * Constructor.
     */
    cSalientRegion();


    //------------------------------------------------------------------------//
    //-------------------------  INTERFACE FUNCTIONS  ------------------------//
    //------------------------------------------------------------------------//

    /**
     * Adds a new point to this cluster.
     *
     * Params:
     * pPoint - a pointer to the point to be added.
     */
    void AddPoint(pcl::PointXYZ* pNewPoint, pcl::Normal* pNormal, bool updateCentroidData=true);

    /**
     * Adds a vector of new points to this cluster.
     *
     * Params:
     * points - A vector containing pointers to the new points.
     */
    void AddPoints(std::vector< std::pair<pcl::PointXYZ*, pcl::Normal*> > points,
				   bool updateCentroidData=true);

    /**
     * Removes the point at the parameter (x,y,z) location from this cluster.
     *
     * Params:
     * x - the x location of the point.
     * y - the y location of the point.
     * z - the z location of the point.
     */
    void RemovePoint(float x, float y, float z, bool updateCentroidData=true);

    /**
     * Returns a pointer to the point at the parameter (x,y,z) location.  Will
     * return null if this cluster does not contain the parameter point.
     *
     * Params:
     * x - the x location of the point.
     * y - the y location of the point.
     * z - the z location of the point.
     */
    pcl::PointXYZ* GetPoint(float x, float y, float z);

    /**
     * Updates the data regarding the centroid of this region: the (x,y,z) location 
     * of the centroid, the local gradient, and the normal orientation.
     */
    void UpdateCentroidData();

	
public:

    //------------------------------------------------------------------------//
    //----------------------------  DATA MEMBERS  ----------------------------//
    //------------------------------------------------------------------------//

    // Contains references to all of the points and their associated normals in this cluster.
    std::vector< std::pair<pcl::PointXYZ*, pcl::Normal*> > mPoints;

    // The location in space of the centroid.  Corresponds to one of the points.
    float mX;
    float mY;
    float mZ;

    // The local gradient term, which corresponds to the average projection of a
    // local surface normal within its r x r pixel neighborhood, Omega. (equation
    // 1 of Asif 2014).
    float mLocalGradient;

    // The orientation of the surface normal from a global perspective (equation 3
    // of Asif 2014).
    float mSurfNormOrientation_rad;
};

#endif // SALIENTREGION_H_INCLUDED


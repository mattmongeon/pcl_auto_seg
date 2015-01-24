#include "salient_region.h"

////////////////////////////////////////////////////////////////////////////////
//  Construction
////////////////////////////////////////////////////////////////////////////////

cSalientRegion::cSalientRegion()
{
    mPoints.clear();
}

////////////////////////////////////////////////////////////////////////////////
//  Interface Functions
////////////////////////////////////////////////////////////////////////////////

void cSalientRegion::AddPoint(pcl::PointXYZ* pNewPoint, bool updateCentroidData)
{
    // Check to see if we already have it first, then add it if it is safe.
    for( std::size_t i = 0; i < mPoints.size(); ++i )
    {
	pcl::PointXYZ* pPt = mPoints[i];
	if( (pPt->x == pNewPoint->x) && 
	    (pPt->y == pNewPoint->y) && 
	    (pPt->z == pNewPoint->z) )
	    return;
    }

    mPoints.push_back( pNewPoint );

    if( updateCentroidData )
	UpdateCentroidData();
}

////////////////////////////////////////////////////////////////////////////////

void cSalientRegion::AddPoints(std::vector<pcl::PointXYZ*> points, bool updateCentroidData)
{
    for( std::size_t i = 0; i < points.size(); ++i )
    {
	// Update centroid data at the end (maybe, depending on the parameter).
	AddPoint(points[i], false);
    }


    if( updateCentroidData )
	UpdateCentroidData();
}

////////////////////////////////////////////////////////////////////////////////

void cSalientRegion::RemovePoint(float x, float y, float z, bool updateCentroidData)
{
    for( std::vector<pcl::PointXYZ*>::iterator it = mPoints.begin();
	 it != mPoints.end();
	 ++it )
    {
	pcl::PointXYZ* pPt = *it;
	if((pPt->x == x) && (pPt->y == y) && (pPt->z == z))
	{
	    mPoints.erase(it);
	    return;
	}
    }

    if( updateCentroidData )
	UpdateCentroidData();
}

////////////////////////////////////////////////////////////////////////////////

pcl::PointXYZ* cSalientRegion::GetPoint(float x, float y, float z)
{
    for( std::size_t i = 0; i < mPoints.size(); ++i )
    {
	pcl::PointXYZ* pPt = mPoints[i];
	if((pPt->x == x) && (pPt->y == y) && (pPt->z == z))
	{
	    return mPoints[i];
	}
    }

    return 0;
}

////////////////////////////////////////////////////////////////////////////////

void cSalientRegion::UpdateCentroidData()
{

}

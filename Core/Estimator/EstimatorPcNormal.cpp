#include <pcl/features/normal_3d_omp.h>

#include "EstimatorPcNormal.h"

namespace RecRoom
{
	void EstimatorPcNormal::Process(
		const PTR(AccMED)& searchMethod,
		const PTR(PcMED)& searchSurface,
		const PTR(PcMED)& inV,
		const PTR(PcIndex)& inIdx,
		PcMED& outV) const
	{
		if (searchMethod->getInputCloud() != searchSurface) // Make sure the tree searches the surface
		{
			PRINT_WARNING("searchMethod is not using searchSurface, set to it.");
			searchMethod->setInputCloud(searchSurface);
		}
		
#ifdef POINT_MED_WITH_NORMAL
		pcl::NormalEstimationOMP<PointMED, PointMED> ne;
		if (searchMethod)
			ne.setSearchMethod(searchMethod);
		ne.setRadiusSearch(searchRadius);
		if (searchSurface)
			ne.setSearchSurface(searchSurface);
		ne.setInputCloud(inV);
		if (inIdx)
			ne.setIndices(inIdx);
		ne.compute(outV);
#endif
	}
}
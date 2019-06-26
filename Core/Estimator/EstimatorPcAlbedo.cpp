#include "Common/AlbedoEstimation.h"

#include "EstimatorPcAlbedo.h"

namespace RecRoom
{
	void EstimatorPcAlbedo::Process(
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
#ifdef POINT_MED_WITH_LABEL
#ifdef POINT_MED_WITH_INTENSITY
		AlbedoEstimationOMP ae(scanner->getScanMetaSet(), linearSolver, distInterParm, angleInterParm, cutFalloff, cutGrazing);
		if (searchMethod)
			ae.setSearchMethod(searchMethod);
		ae.setRadiusSearch(searchRadius);
		if (searchSurface)
			ae.setSearchSurface(searchSurface);
		ae.setInputCloud(inV);
		if (inIdx)
			ae.setIndices(inIdx);
		ae.compute(outV);
#endif
#endif
#endif
	}
}
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

		AlbedoEstimationOMP ae(scanMeta, linearSolver, distInterParm, angleInterParm, cutFalloff, cutGrazing);
		if (searchMethod)
			ae.setSearchMethod(searchMethod);
		ae.setRadiusSearch(searchRadius);
		if (searchSurface)
			ae.setSearchSurface(searchSurface);
		ae.setInputCloud(inV);
		if (inIdx)
			ae.setIndices(inIdx);
		ae.compute(outV);
	}
}
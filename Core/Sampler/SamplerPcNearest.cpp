#include "SamplerPcNearest.h"

namespace RecRoom
{
	void SamplerPcNearest::Process(
		const PTR(AccMED)& searchMethod,
		const PTR(PcMED)& searchSurface,
		const PTR(PcMED)& inV,
		PcIndex& outV) const
	{
		outV.resize(inV->size());

		if (searchMethod->getInputCloud() != searchSurface) // Make sure the tree searches the surface
		{
			PRINT_WARNING("searchMethod is not using searchSurface, set to it.");
			searchMethod->setInputCloud(searchSurface);
		}

#ifdef _OPENMP
#pragma omp parallel for num_threads(omp_get_num_procs())
#endif
		// Iterating over the entire index vector
		for (int px = 0; px < static_cast<int> (inV->size()); ++px)
		{
			std::vector<int> ki;
			std::vector<float> kd;
			if (searchMethod->nearestKSearch(*inV, px, 1, ki, kd) > 0)
				outV[px] = ki[0];
			else
				outV[px] = -1;
		}
	}
}
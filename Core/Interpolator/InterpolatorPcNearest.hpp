#pragma once

#include "InterpolatorPcNearest.h"

namespace RecRoom
{
	template<class InPointType, class OutPointType>
	void InterpolatorPcNearest<InPointType, OutPointType>::ImplementProcess(
		const CONST_PTR(Acc<InPointType>)& searchSurface,
		const CONST_PTR(Pc<InPointType>)& input,
		const CONST_PTR(PcIndex)& filter,
		Pc<OutPointType>& output) const
	{
		output.clear();

		output.resize(filter->size());
#ifdef _OPENMP
#pragma omp parallel for num_threads(numThreads)
#endif
		// Iterating over the entire index vector
		for (int idx = 0; idx < static_cast<int> (filter->size()); ++idx)
		{
			int px = (*filter)[idx];
			std::vector<int> ki;
			std::vector<float> kd;
			if (searchSurface->nearestKSearch(*input, px, 1, ki, kd) > 0)
			{
				output[idx] = (*searchSurface->getInputCloud())[ki[0]];
				output[idx].x = (*input)[px].x;
				output[idx].y = (*input)[px].y;
				output[idx].z = (*input)[px].z;
			}
			else
			{
				output[idx] = (*input)[px];
			}
		}
	}
}
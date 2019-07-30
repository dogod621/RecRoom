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

		std::vector<int> nnIndices;
		std::vector<float> nnSqrDists;
#ifdef _OPENMP
#pragma omp parallel for private (nnIndices, nnSqrDists) num_threads(numThreads)
#endif
		// Iterating over the entire index vector
		for (int idx = 0; idx < static_cast<int> (filter->size()); ++idx)
		{
			int px = (*filter)[idx];
			if (searchSurface->nearestKSearch(*input, px, 1, nnIndices, nnSqrDists) > 0)
			{
				const InPointType& p = (*searchSurface->getInputCloud())[nnIndices[0]];
				output[idx] = p;
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
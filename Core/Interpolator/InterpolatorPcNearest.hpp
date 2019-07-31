#pragma once

#include "InterpolatorPcNearest.h"

namespace RecRoom
{
	template<class InPointType, class OutPointType>
	void InterpolatorPcNearest<InPointType, OutPointType>::InterpolationTask(
		int id,
		void* self_,
		void* searchSurface_,
		void* input_,
		void* filter_,
		void* output_)
	{
		InterpolatorPcNearest<InPointType, OutPointType>& self = (*(InterpolatorPcNearest<InPointType, OutPointType>*)(self_));
		const Acc<InPointType>& searchSurface = (*(Acc<InPointType>*)(searchSurface_));
		const Pc<InPointType>& input = (*(Pc<InPointType>*)(input_));
		const PcIndex& filter = (*(PcIndex*)(filter_));
		Pc<OutPointType>& output = (*(Pc<OutPointType>*)(output_));

		std::vector<int> nnIndices;
		std::vector<float> nnSqrDists;

		for (int idx = id; idx < static_cast<int> (filter.size()); idx += self.numThreads)
		{
			const InPointType& inPoint = input[filter[idx]];
			OutPointType& outPoint = output[idx];

			if (searchSurface.nearestKSearch(inPoint, 1, nnIndices, nnSqrDists) > 0)
			{
				outPoint = (*searchSurface.getInputCloud())[nnIndices[0]];
				outPoint.x = inPoint.x;
				outPoint.y = inPoint.y;
				outPoint.z = inPoint.z;
			}
			else
			{
				outPoint = inPoint;
			}
		}
	}

	template<class InPointType, class OutPointType>
	void InterpolatorPcNearest<InPointType, OutPointType>::ImplementProcess(
		const CONST_PTR(Acc<InPointType>)& searchSurface,
		const CONST_PTR(Pc<InPointType>)& input,
		const CONST_PTR(PcIndex)& filter,
		Pc<OutPointType>& output) const
	{
#ifdef _OPENMP
		std::vector<int> nnIndices;
		std::vector<float> nnSqrDists;

#pragma omp parallel for private (nnIndices, nnSqrDists) num_threads(numThreads)
		for (int idx = 0; idx < static_cast<int> (filter->size()); ++idx)
		{
			const InPointType& inPoint = (*input)[(*filter)[idx]];
			OutPointType& outPoint = output[idx];

			if (searchSurface->nearestKSearch(inPoint, 1, nnIndices, nnSqrDists) > 0)
			{
				outPoint = (*searchSurface->getInputCloud())[nnIndices[0]];
				outPoint.x = inPoint.x;
				outPoint.y = inPoint.y;
				outPoint.z = inPoint.z;
			}
			else
			{
				outPoint = inPoint;
			}
		}
#else
		PRINT_WARNING("OPENMP is not enabled, use std thread instead");
		std::vector<std::thread> threads;
		for (int i = 0; i < numThreads; i++)
			threads.push_back(std::thread(InterpolatorPcNearest::InterpolationTask, i,
			(void*)(this),
				(void*)(&(*searchSurface)),
				(void*)(&(*input)),
				(void*)(&(*filter)),
				(void*)(&output)));
		for (auto& thread : threads)
			thread.join();
#endif
	}
}
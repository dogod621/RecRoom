#pragma once

#include "FilterPcRemoveDuplicate.h"

namespace RecRoom
{
	template<class PointType>
	void FilterPcRemoveDuplicate<PointType>::ImplementProcess(
		const CONST_PTR(Acc<PointType>)& searchSurface,
		const CONST_PTR(Pc<PointType>)& input,
		const CONST_PTR(PcIndex)& filter,
		PcIndex& output) const
	{
		output.clear();

		std::vector<bool> pcStatus;
		pcStatus.resize(input->size());
		output.reserve(filter->size());

		for (std::size_t px = 0; px < pcStatus.size(); ++px)
			pcStatus[px] = false;
		for (PcIndex::const_iterator it = filter->begin(); it != filter->end(); ++it)
			pcStatus[(*it)] = true;

		for (PcIndex::const_iterator it = filter->begin(); it != filter->end(); ++it)
		{
			if (pcStatus[*it])
			{
				output.push_back(*it);
				pcStatus[*it] = false;

				std::vector<int> ki;
				std::vector<float> kd;
				int k = searchSurface->radiusSearch(*input, *it, minDistance, ki, kd);
				for (int i = 0; i < k; ++i)
					pcStatus[ki[i]] = false;
			}
		}
	}
}
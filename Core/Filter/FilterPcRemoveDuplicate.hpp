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
		if (filter)
		{
			output.reserve(filter->size());

			for (std::size_t px = 0; px < pcStatus.size(); ++px)
				pcStatus[px] = false;

			for (PcIndex::const_iterator it = filter->begin(); it != filter->end(); ++it)
				pcStatus[(*it)] = true;
		}
		else
		{
			output.reserve(input->size());

			for (std::size_t px = 0; px < pcStatus.size(); ++px)
				pcStatus[px] = true;
		}

		for (int px = 0; px < input->size(); ++px)
		{
			if (pcStatus[px])
			{
				output.push_back(px);
				pcStatus[px] = false;

				std::vector<int> ki;
				std::vector<float> kd;
				int k = searchSurface->radiusSearch(*input, px, minDistance, ki, kd);
				for (int i = 0; i < k; ++i)
					pcStatus[ki[i]] = false;
			}
		}
	}
}
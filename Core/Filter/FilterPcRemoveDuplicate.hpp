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
		std::vector<int> dfsStack;
		if (filter)
		{
			dfsStack.reserve(filter->size());
			output.reserve(filter->size());

			for (std::size_t px = 0; px < pcStatus.size(); ++px)
				pcStatus[px] = false;

			for (PcIndex::const_iterator it = filter->begin(); it != filter->end(); ++it)
			{
				if((*it) >= 0) 
					pcStatus[(*it)] = true;
			}
		}
		else
		{
			dfsStack.reserve(input->size());
			output.reserve(input->size());

			for (std::size_t px = 0; px < pcStatus.size(); ++px)
				pcStatus[px] = true;
		}

		{
			PRINT_INFO("DFS Search - Start");

			for (int px1 = 0; px1 < input->size(); ++px1)
			{
				if (pcStatus[px1])
				{
					dfsStack.push_back(px1);
					pcStatus[px1] = false;
				}

				while (!dfsStack.empty())
				{
					int px2 = dfsStack.back();
					if (pcStatus[px2])
						THROW_EXCEPTION("This should not happened");

					dfsStack.pop_back();
					output.push_back(px2);

					std::vector<int> ki;
					std::vector<float> kd;
					if (searchSurface->radiusSearch(*input, px2, searchRadius, ki, kd) > 1)
					{
						for (std::size_t i = 0; i < ki.size(); ++i)
						{
							int px3 = ki[i];
							if ((kd[i] > sqrMinDistance) && pcStatus[px3])
								dfsStack.push_back(px3);
							pcStatus[px3] = false;
						}
					}
				}
			}

			PRINT_INFO("DFS Search - End");
		}
	}
}
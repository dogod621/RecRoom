#pragma once

#include "FilterPcRemoveNonFinite.h"

namespace RecRoom
{
	template<class PointType>
	void FilterPcRemoveNonFinite<PointType>::ImplementProcess(
		const CONST_PTR(Acc<PointType>)& searchSurface,
		const CONST_PTR(Pc<PointType>)& input,
		const CONST_PTR(PcIndex)& filter,
		PcIndex& output) const
	{
		output.clear();

		if (filter)
		{
			output.reserve(filter->size());
			for (PcIndex::const_iterator it = filter->begin(); it != filter->end(); ++it)
			{
				if (pcl::isFinite((*input)[(*it)]))
					output.push_back(*it);
			}
		}
		else
		{
			output.reserve(input->size());
			for (int px = 0 ; px < input->size(); ++px)
			{
				if (pcl::isFinite((*input)[px]))
					output.push_back(px);
			}
		}
	}
}
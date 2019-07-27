#pragma once

#include "FilterPcAABB.h"

namespace RecRoom
{
	template<class PointType>
	void FilterPcAABB<PointType>::ImplementProcess(
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
				const PointType& intP = (*input)[*it];

				if (pcl::isFinite(intP))
				{
					if ((intP.x >= minAABB[0]) && (intP.y >= minAABB[1]) && (intP.z >= minAABB[2]) &&
						(intP.x < maxAABB[0]) && (intP.y < maxAABB[1]) && (intP.z < maxAABB[2]))
						output.push_back(*it);
				}
			}
		}
		else
		{
			output.reserve(input->size());

			for (int px = 0; px < input->size(); ++px)
			{
				const PointType& intP = (*input)[px];

				if (pcl::isFinite(intP))
				{
					if ((intP.x >= minAABB[0]) && (intP.y >= minAABB[1]) && (intP.z >= minAABB[2]) &&
						(intP.x < maxAABB[0]) && (intP.y < maxAABB[1]) && (intP.z < maxAABB[2]))
						output.push_back(px);
				}
			}
		}
	}
}
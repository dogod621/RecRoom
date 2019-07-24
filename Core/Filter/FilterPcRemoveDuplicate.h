#pragma once

#include "FilterPc.h"

namespace RecRoom
{
	template<class PointType>
	class FilterPcRemoveDuplicate : public FilterPc<PointType>
	{
	public:
		FilterPcRemoveDuplicate(double searchRadius, float minDistance)
			: FilterPc<PointType>(), searchRadius(searchRadius), minDistance(minDistance), sqrMinDistance(minDistance * minDistance)
		{
			if (searchRadius <= minDistance)
				THROW_EXCEPTION("searchRadius <= minDistance");
		}

	public:
		virtual void ImplementProcess(
			const CONST_PTR(Acc<PointType>)& searchSurface,
			const CONST_PTR(Pc<PointType>)& input,
			const CONST_PTR(PcIndex)& filter,
			PcIndex& output) const;

	public:
		double getSearchRadius() const { return searchRadius; }
		float getMinDistance() const { return minDistance; }

		void setSearchRadius(double v) 
		{
			if (v <= minDistance)
				PRINT_WARNING("v <= minDistance, ignore"); 
			else
				searchRadius = v; 
		}

		void setMinDistance(float v) 
		{ 
			if (v > searchRadius)
				PRINT_WARNING("v > searchRadius, ignore");
			else
			{
				minDistance = v;
				sqrMinDistance = v * v;
			}
		}
		
	protected:
		double searchRadius;
		float minDistance;
		float sqrMinDistance;
	};
}

#include "FilterPcRemoveDuplicate.hpp"
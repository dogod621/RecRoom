#pragma once

#include "FilterPc.h"

namespace RecRoom
{
	template<class PointType>
	class FilterPcRemoveDuplicate : public FilterPc<PointType>
	{
	public:
		FilterPcRemoveDuplicate(float minDistance)
			: FilterPc<PointType>(),  minDistance(minDistance)
		{
			name = "FilterPcRemoveDuplicate";
		}

	protected:
		virtual void ImplementProcess(
			const CONST_PTR(Acc<PointType>)& searchSurface,
			const CONST_PTR(Pc<PointType>)& input,
			const CONST_PTR(PcIndex)& filter,
			PcIndex& output) const;

	public:
		inline virtual bool SearchPointValid(const PointType& p) const
		{
			PRINT_WARNING("not used");
			return true;
		}

	public:
		float getMinDistance() const { return minDistance; }
		void setMinDistance(float v) { minDistance = v; }
		
	protected:
		float minDistance;
	};
}

#include "FilterPcRemoveDuplicate.hpp"
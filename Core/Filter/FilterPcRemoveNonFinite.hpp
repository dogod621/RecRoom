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
		output = (*filter);
	}
}
#pragma once

#include "SamplerPcMLS.h"

namespace RecRoom
{
	template<class PointType>
	void SamplerPcMLS<PointType>::ImplementProcess(
		const CONST_PTR(Acc<PointType>)& searchSurface,
		const CONST_PTR(Pc<PointType>)& input,
		const CONST_PTR(PcIndex)& filter,
		Pc<PointType> & output) const
	{
		MovingLeastSquares<PointType, PointType> mls(searchRadius, order, projectionMethod, upsampleMethod, threads, computeNormals);

		mls.setSearchMethod(boost::const_pointer_cast<Acc<PointType>>(searchSurface)); // trick, already ensure it won't be modified 

		if (filter)
			mls.setIndices(filter);

		mls.process(output);
	}
}
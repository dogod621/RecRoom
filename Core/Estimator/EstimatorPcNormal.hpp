#pragma once

#include "Common/NormalEstimation.h"

#include "EstimatorPcNormal.h"

namespace RecRoom
{
	template<class InPointType, class OutPointType>
	void EstimatorPcNormal<InPointType, OutPointType>::ImplementProcess(
		const CONST_PTR(Acc<InPointType>)& searchSurface,
		const CONST_PTR(Pc<InPointType>)& input,
		const CONST_PTR(PcIndex)& filter,
		Pc<OutPointType>& output) const
	{
		NormalEstimation<InPointType, OutPointType> ne (scanner, distInterParm, cutFalloff);

		ne.setInputCloud(input);

		ne.setSearchMethod(boost::const_pointer_cast<Acc<InPointType>>(searchSurface)); // trick, already ensure it won't be modified 
		ne.setRadiusSearch(searchRadius);
		ne.setSearchSurface(searchSurface->getInputCloud());
		ne.setIndices(filter);

		ne.compute(output);
	}
}
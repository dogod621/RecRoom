#pragma once

#include "Common/NDFEstimation.h"

#include "EstimatorPcNDF.h"

namespace RecRoom
{
	template<class InPointType, class OutPointType>
	void EstimatorPcNDF<InPointType, OutPointType>::ImplementProcess(
		const CONST_PTR(Acc<InPointType>)& searchSurface,
		const CONST_PTR(Pc<InPointType>)& input,
		const CONST_PTR(PcIndex)& filter,
		Pc<OutPointType>& output) const
	{
		NDFEstimation<InPointType, OutPointType> fe(scanner, linearSolver, distInterParm, angleInterParm, cutFalloff, cutGrazing);

		fe.setInputCloud(input);
		fe.setSearchMethod(boost::const_pointer_cast<Acc<InPointType>>(searchSurface)); // trick, already ensure it won't be modified 
		fe.setRadiusSearch(searchRadius);
		fe.setSearchSurface(searchSurface->getInputCloud());

		if (filter)
			fe.setIndices(filter);

		fe.compute(output);
	}
}
#pragma once

#include "Common/AlbedoEstimation.h"

#include "EstimatorPcAlbedo.h"

namespace RecRoom
{
	template<class InPointType, class OutPointType>
	void EstimatorPcAlbedo<InPointType, OutPointType>::ImplementProcess(
		const CONST_PTR(Acc<InPointType>)& searchSurface,
		const CONST_PTR(Pc<InPointType>)& input,
		const CONST_PTR(PcIndex)& filter,
		Pc<OutPointType>& output) const
	{
		AlbedoEstimation<InPointType, OutPointType> ae(scanner, linearSolver, distInterParm, angleInterParm, cutFalloff, cutGrazing);

		ae.setInputCloud(input);
		ae.setSearchMethod(boost::const_pointer_cast<Acc<InPointType>>(searchSurface)); // trick, already ensure it won't be modified 
		ae.setRadiusSearch(searchRadius);
		ae.setSearchSurface(searchSurface->getInputCloud());
		ae.setIndices(filter);

		ae.compute(output);
	}
}
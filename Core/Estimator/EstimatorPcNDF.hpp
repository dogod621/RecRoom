#pragma once

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
		switch (ndf)
		{
		case NDF::SG:
		{
			SGEstimation<InPointType, OutPointType> fe(scanner, distInterParm, angleInterParm, cutFalloff, cutGrazing);

			fe.setInputCloud(input);
			fe.setSearchMethod(boost::const_pointer_cast<Acc<InPointType>>(searchSurface)); // trick, already ensure it won't be modified 
			fe.setRadiusSearch(searchRadius);
			fe.setSearchSurface(searchSurface->getInputCloud());
			fe.setIndices(filter);

			fe.compute(output);
		}
		break;
		default:
		{
			THROW_EXCEPTION("ndf is not support");
		}
		break;
		}
	}
}
#pragma once

#include "SamplerPcMLS.h"

namespace RecRoom
{
	template<class PointType>
	void SamplerPcMLS<PointType>::Process(
		const PTR(Pc<PointType>) & inV,
		Pc<PointType> & outV) const
	{
		PTR<KDTree<PointType>> tree(new KDTree<PointType>);

		MovingLeastSquares<PointType, PointType> mls(searchRadius, order, projectionMethod, upsampleMethod, threads, computeNormals);
		mls.setInputCloud(inV);
		mls.setSearchMethod(tree);
		mls.process(outV);
	}
}
#pragma once

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/impl/statistical_outlier_removal.hpp>

#include "FilterPcRemoveOutlier.h"

namespace RecRoom
{
	template<class PointType>
	void FilterPcRemoveOutlier<PointType>::ImplementProcess(
		const CONST_PTR(Acc<PointType>)& searchSurface,
		const CONST_PTR(Pc<PointType>)& input,
		const CONST_PTR(PcIndex)& filter,
		PcIndex& output) const
	{
		pcl::StatisticalOutlierRemoval<PointType> olr;
		olr.setMeanK(meanK);
		olr.setStddevMulThresh(stdMul);
		olr.setInputCloud(input);
		olr.setSearchMethod(searchSurface);

		if (filter)
			olr.setIndices(filter);

		olr.filter(output);
	}
}

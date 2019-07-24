#pragma once

#include <pcl/filters/crop_box.h>
#include <pcl/filters/impl/crop_box.hpp>

#include "FilterPcAABB.h"

namespace RecRoom
{
	template<class PointType>
	void FilterPcAABB<PointType>::ImplementProcess(
		const CONST_PTR(Acc<PointType>)& searchSurface,
		const CONST_PTR(Pc<PointType>)& input,
		const CONST_PTR(PcIndex)& filter,
		PcIndex& output) const
	{
		pcl::CropBox<PointMED> cb;
		cb.setMin(Eigen::Vector4f(minAABB.x(), minAABB.y(), minAABB.z(), 1.0));
		cb.setMax(Eigen::Vector4f(maxAABB.x(), maxAABB.y(), maxAABB.z(), 1.0));
		cb.setInputCloud(input);

		if (filter)
			cb.setIndices(filter);

		cb.filter(output);
	}
}
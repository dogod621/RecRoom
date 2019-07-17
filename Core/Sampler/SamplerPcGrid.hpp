#pragma once

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/impl/voxel_grid.hpp>

#include "SamplerPcGrid.h"

namespace RecRoom
{
	template<class PointType>
	void SamplerPcGrid<PointType>::Process(
		const PTR(Pc<PointType>) & inV,
		Pc<PointType> & outV) const
	{
		pcl::VoxelGrid<PointType> vf;
		vf.setLeafSize(voxelSize, voxelSize, voxelSize);
		vf.setInputCloud(inV);
		vf.filter(outV);
	}
}
#pragma once

#include <pcl/surface/grid_projection.h>
#include <pcl/surface/impl/grid_projection.hpp>

#include "MesherPcGP.h"

namespace RecRoom
{
	template<class PointType>
	void MesherPcGP<PointType>::ToMesh(PTR(Acc<PointType>)& searchSurface, PTR(Pc<PointType>)& input, Mesh& output) const
	{
		pcl::GridProjection<PointType> gp(resolution);

		gp.setMaxBinarySearchLevel(maxBinarySearchLevel);
		gp.setNearestNeighborNum(maxNumNei);
		gp.setPaddingSize(paddingSize);

		gp.setInputCloud(input);
		gp.setSearchMethod(searchSurface);
		gp.reconstruct(output);
	}
}
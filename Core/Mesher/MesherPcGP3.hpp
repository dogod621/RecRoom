#pragma once

#include <pcl/surface/gp3.h>
#include <pcl/surface/impl/gp3.hpp>

#include "MesherPcGP3.h"

namespace RecRoom
{
	template<class PointType>
	void MesherPcGP3<PointType>::ToMesh(PTR(Acc<PointType>)& searchSurface, PTR(Pc<PointType>)& input, Mesh& output) const
	{
		pcl::GreedyProjectionTriangulation<PointType> gp3;

		gp3.setSearchRadius(searchRadius);
		gp3.setMu(mu);
		gp3.setMaximumNearestNeighbors(maxNumNei);
		gp3.setMinimumAngle(minAngle);
		gp3.setMaximumAngle(maxAngle);
		gp3.setMaximumSurfaceAngle(epsAngle);
		gp3.setNormalConsistency(consistent);
		gp3.setConsistentVertexOrdering(consistentOrdering);

		gp3.setInputCloud(input);
		gp3.setSearchMethod(searchSurface);
		gp3.reconstruct(output);
	}
}
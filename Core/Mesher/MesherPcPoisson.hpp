#pragma once

#include <pcl/surface/poisson.h>
#include <pcl/surface/impl/poisson.hpp>

#include "MesherPcPoisson.h"

namespace RecRoom
{
	template<class PointType>
	void MesherPcPoisson<PointType>::ToMesh(PTR(Acc<PointType>)& searchSurface, PTR(Pc<PointType>)& input, Mesh& output) const
	{
		pcl::Poisson<pcl::PointNormal> pn;

		pn.setDepth(depth);
		pn.setMinDepth(minDepth);
		pn.setPointWeight(pointWeight);
		pn.setScale(scale);
		pn.setSolverDivide(solverDivide);
		pn.setIsoDivide(isoDivide);
		pn.setSamplesPerNode(samplesPerNode);
		pn.setConfidence(confidence);
		pn.setOutputPolygons(outputPolygons);
		pn.setManifold(manifold);
		pn.setDegree(degree);

		pn.setInputCloud(input);
		pn.setSearchMethod(searchSurface);
		pn.reconstruct(output);
	}
}
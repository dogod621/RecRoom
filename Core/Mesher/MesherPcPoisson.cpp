#include <pcl/surface/poisson.h>

#include "MesherPcPoisson.h"

namespace RecRoom
{
	void MesherPcPoisson::ToMesh(PTR(Pc<pcl::PointNormal>)& inV, PTR(KDTree<pcl::PointNormal>)& tree, pcl::PolygonMesh& out) const
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

		pn.setInputCloud(inV);
		pn.setSearchMethod(tree);
		pn.reconstruct(out);
	}
}
#include <pcl/surface/gp3.h>

#include "MesherPcGP3.h"

namespace RecRoom
{
	void MesherPcGP3::ToMesh(PTR(PcREC)& inV, PTR(KDTreeREC)& tree, pcl::PolygonMesh& out) const
	{
		pcl::GreedyProjectionTriangulation<PointREC> gp3;
		
		gp3.setSearchRadius(searchRadius);
		gp3.setMu(mu);
		gp3.setMaximumNearestNeighbors(maxNumNei);
		gp3.setMinimumAngle(minAngle);
		gp3.setMaximumAngle(maxAngle);
		gp3.setMaximumSurfaceAngle(epsAngle);
		gp3.setNormalConsistency(consistent);
		gp3.setConsistentVertexOrdering(consistentOrdering);

		gp3.setInputCloud(inV);
		gp3.setSearchMethod(tree);
		gp3.reconstruct(out);
	}
}
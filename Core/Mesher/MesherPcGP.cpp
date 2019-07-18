#include <pcl/surface/grid_projection.h>
#include <pcl/surface/impl/grid_projection.hpp>

#include "MesherPcGP.h"

namespace RecRoom
{
	void MesherPcGP::ToMesh(PTR(PcREC)& inV, PTR(KDTreeREC)& tree, pcl::PolygonMesh& out) const
	{
		pcl::GridProjection<PointREC> gp (resolution);

		gp.setMaxBinarySearchLevel(maxBinarySearchLevel);
		gp.setNearestNeighborNum(maxNumNei);
		gp.setPaddingSize(paddingSize);

		gp.setInputCloud(inV);
		gp.setSearchMethod(tree);
		gp.reconstruct(out);
	}
}
#include <pcl/surface/grid_projection.h>
#include <pcl/surface/impl/grid_projection.hpp>

#include "MesherPcGP.h"

namespace RecRoom
{
	void MesherPcGP::ToMesh(PTR(Pc<pcl::PointNormal>)& inV, PTR(KDTree<pcl::PointNormal>)& tree, pcl::PolygonMesh& out) const
	{
		pcl::GridProjection<pcl::PointNormal> gp (resolution);

		gp.setMaxBinarySearchLevel(maxBinarySearchLevel);
		gp.setNearestNeighborNum(maxNumNei);
		gp.setPaddingSize(paddingSize);

		gp.setInputCloud(inV);
		gp.setSearchMethod(tree);
		gp.reconstruct(out);
	}
}
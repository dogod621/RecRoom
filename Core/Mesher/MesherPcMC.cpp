#include <pcl/search/kdtree.h>

#include "Common/MarchingCubesRBF.h"
#include "Common/MarchingCubesHoppe.h"
#include "Sampler/SamplerPcNearest.h"

#include "MesherPcMC.h"

namespace RecRoom
{
	void MesherPcMCRBF::ToMesh(PTR(Pc<pcl::PointNormal>)& inV, PTR(KDTree<pcl::PointNormal>)& tree, pcl::PolygonMesh& out) const
	{
		MarchingCubesRBF<pcl::PointNormal> mc(offSurfaceEpsilon, percentageExtendGrid, isoLevel, gridRes, gridRes, gridRes);
		mc.setInputCloud(inV);
		mc.setSearchMethod(tree);
		mc.reconstruct(out);
	}

	void MesherPcMCHoppe::ToMesh(PTR(Pc<pcl::PointNormal>)& inV, PTR(KDTree<pcl::PointNormal>)& tree, pcl::PolygonMesh& out) const
	{
		MarchingCubesHoppe<pcl::PointNormal> mc(distIgnore, percentageExtendGrid, isoLevel, gridRes, gridRes, gridRes);
		mc.setInputCloud(inV);
		mc.setSearchMethod(tree);
		mc.reconstruct(out);
	}
}
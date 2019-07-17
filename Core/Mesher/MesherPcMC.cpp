#include <pcl/search/kdtree.h>

#include "Common/MarchingCubesRBF.h"
#include "Common/MarchingCubesHoppe.h"
#include "Sampler/SamplerPcNearest.h"

#include "MesherPcMC.h"

namespace RecRoom
{
	void MesherPcMCRBF::ToMesh(PTR(PcREC)& inV, PTR(KDTreeREC)& tree, pcl::PolygonMesh& out) const
	{
		MarchingCubesRBF mc(offSurfaceEpsilon, percentageExtendGrid, isoLevel, gridRes, gridRes, gridRes);
		mc.setInputCloud(inV);
		mc.setSearchMethod(tree);
		mc.reconstruct(out);
	}

	void MesherPcMCHoppe::ToMesh(PTR(PcREC)& inV, PTR(KDTreeREC)& tree, pcl::PolygonMesh& out) const
	{
		MarchingCubesHoppe mc(distIgnore, percentageExtendGrid, isoLevel, gridRes, gridRes, gridRes);
		mc.setInputCloud(inV);
		mc.setSearchMethod(tree);
		mc.reconstruct(out);
	}
}
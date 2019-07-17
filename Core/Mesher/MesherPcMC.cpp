#include <pcl/search/kdtree.h>

#include "Common/MarchingCubesRBF.h"
#include "Common/MarchingCubesHoppe.h"
#include "Sampler/SamplerPcNearest.h"

#include "MesherPcMC.h"

namespace RecRoom
{
	void MesherPcMC::Process(PTR(PcMED)& inV, pcl::PolygonMesh& out) const
	{

		PTR(PcREC) pc(new PcREC);
		PTR(KDTreeREC) tree(new KDTreeREC);

		pc->reserve(inV->size());
		for (PcMED::const_iterator it = inV->begin(); it != inV->end(); ++it)
		{
			if (pcl::isFinite(*it))
				pc->push_back(*it);
		}

		ToMesh(pc, tree, out);

		PTR(PcREC) temp (new PcREC);
		pcl::fromPCLPointCloud2(out.cloud, *temp);

		SamplerPcNearestREC upSampler;
		PcIndex upIdx;
		upSampler.Process(tree, pc, temp, upIdx);

		for (std::size_t px = 0; px < upIdx.size(); ++px)
		{
			if (upIdx[px] >= 0)
			{
				PointREC& tarP = (*temp)[px];
				PointREC& srcP = (*pc)[upIdx[px]];

#ifdef POINT_REC_WITH_NORMAL
				tarP.normal_x = srcP.normal_x;
				tarP.normal_y = srcP.normal_y;
				tarP.normal_z = srcP.normal_z;
				tarP.curvature = srcP.curvature;
#endif

#ifdef POINT_REC_WITH_RGB
				tarP.r = srcP.r;
				tarP.g = srcP.g;
				tarP.b = srcP.b;
#endif

#ifdef POINT_REC_WITH_INTENSITY
				tarP.intensity = srcP.intensity;
				tarP.intensity = srcP.intensity;
				tarP.intensity = srcP.intensity;
#endif
#ifdef POINT_REC_WITH_LABEL
				tarP.label = srcP.label;
#endif		
			}
		}

		pcl::toPCLPointCloud2(*temp, out.cloud);
	}

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
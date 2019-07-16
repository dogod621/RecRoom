#include "MesherPcMCRBF.h"

#include <pcl/search/kdtree.h>
#include <pcl/surface/marching_cubes_rbf.h>

namespace RecRoom
{
	void MesherPcMCRBF::Process(PTR(PcMED)& inV, pcl::PolygonMesh& out) const
	{
		PRINT_INFO("To PcREC - Start");

		PTR(PcREC) pc(new PcREC);
		pc->reserve(inV->size());
		for (PcMED::const_iterator it = inV->begin(); it != inV->end(); ++it)
		{
			if (pcl::isFinite(*it))
				pc->push_back(*it);
		}

		PRINT_INFO("To PcREC - End");

		PRINT_INFO("Build Acc - Start");

		pcl::search::KdTree<PointREC>::Ptr tree(new pcl::search::KdTree<PointREC>);
		tree->setInputCloud(pc);

		PRINT_INFO("Build Acc - End");

		PRINT_INFO("Rec - Start");

		pcl::MarchingCubesRBF<PointREC> mc (offSurfaceEpsilon, percentageExtendGrid, isoLevel);
		mc.setInputCloud(pc);
		mc.setSearchMethod(tree);

		mc.reconstruct(out);
		PRINT_INFO("Rec - End");

	}
}
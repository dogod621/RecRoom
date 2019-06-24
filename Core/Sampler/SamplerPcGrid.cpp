#include <pcl/filters/voxel_grid.h>

#include "SamplerPcGrid.h"

namespace RecRoom
{
	void SamplerPcGrid::Process(
		const PTR(PcMED) & inV,
		PcMED & outV) const
	{
		pcl::VoxelGrid<PointType> vf;
		vf.setLeafSize(voxelSize, voxelSize, voxelSize);
		vf.setInputCloud(inV);
		vf.filter(outV);
	}
}
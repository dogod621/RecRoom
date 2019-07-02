#pragma once

#include "SupervoxelClustering.h"

namespace pcl
{
	namespace octree
	{
		template<> void pcl::octree::OctreePointCloudAdjacencyContainer<RecRoom::PointMED, RecRoom::Voxel>::addPoint(const RecRoom::PointMED& point)
		{
			++num_points_;
			data_ += point;
		}

		template<> void pcl::octree::OctreePointCloudAdjacencyContainer<RecRoom::PointMED, RecRoom::Voxel>::computeData()
		{
			data_ /= (static_cast<float> (num_points_));
		}
	}
}

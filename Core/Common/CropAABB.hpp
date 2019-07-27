#pragma once

#include <pcl/common/io.h>
#include <pcl/common/impl/io.hpp>

#include "CropAABB.h"

namespace RecRoom
{
	template<typename PointT> 
	void CropAABB<PointT>::applyFilter(PointCloud& output)
	{
		std::vector<int> indices;
		if (keep_organized_)
		{
			THROW_EXCEPTION("Not support.");
		}
		else
		{
			output.is_dense = true;
			applyFilter(indices);
			pcl::copyPointCloud(*input_, indices, output);
		}
	}

	template<typename PointT> 
	void CropAABB<PointT>::applyFilter(PcIndex& indices)
	{
		if (negative_)
		{
			THROW_EXCEPTION("Not support.");
		}

		indices.clear();
		indices.reserve(input_->points.size());

		for (PcIndex::const_iterator it = indices_->begin(); it != indices_->end(); ++it)
		{
			const PointT& intP = input_->points[*it];

			if (pcl::isFinite(intP))
			{
				// If outside the cropbox
				if ((intP.x >= minAABB[0]) && (intP.y >= minAABB[1]) && (intP.z >= minAABB[2]) &&
					(intP.x < maxAABB[0]) && (intP.y < maxAABB[1]) && (intP.z < maxAABB[2]))
					indices.push_back(*it);
			}
		}
	}
}
#pragma once

#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/common/io.h>

#include "VoxelGridFilter.h"

namespace RecRoom
{
	struct CloudPointIndexIdx
	{
		std::size_t idx;
		std::size_t idy;
		std::size_t idz;
		std::size_t cloudPointIndex;

		CloudPointIndexIdx(std::size_t idx, std::size_t idy, std::size_t idz, std::size_t cloudPointIndex) 
			: idx(idx), idy(idy), idz(idz), cloudPointIndex(cloudPointIndex) {}
		bool operator < (const CloudPointIndexIdx& p) const 
		{ 
			if (idx < p.idx)
				return true;
			else if(idx > p.idx)
				return false;
			else
			{
				if (idy < p.idy)
					return true;
				else if (idy > p.idy)
					return false;
				else
				{
					if (idz < p.idz)
						return true;
					else 
						return false;
				}
			}
		}

		bool operator == (const CloudPointIndexIdx& p) const
		{
			return (idx == p.idx) && (idy == p.idy) && (idz == p.idz);
		}
	};

	template <typename PointT> 
	void VoxelGridFilter<PointT>::applyFilter(PointCloud& output)
	{
		std::vector<CloudPointIndexIdx> indices;
		indices.reserve(indices_->size());

		for (std::vector<int>::const_iterator it = indices_->begin(); it != indices_->end(); ++it)
		{
			if ((*it) >= 0)
			{
				const PointT& p = (*input_)[(*it)];

				if (pcl_isfinite(p.x) &&
					pcl_isfinite(p.y) &&
					pcl_isfinite(p.z))
				{
					if ((p.x > minAABB.x()) &&
						(p.y > minAABB.y()) &&
						(p.z > minAABB.z()) &&
						(p.x < maxAABB.x()) &&
						(p.y < maxAABB.y()) &&
						(p.z < maxAABB.z()))
					{
						std::size_t idx = static_cast<std::size_t> (std::floor((p.x - minAABB.x()) * invLeafSize[0]));
						std::size_t idy = static_cast<std::size_t> (std::floor((p.y - minAABB.y()) * invLeafSize[1]));
						std::size_t idz = static_cast<std::size_t> (std::floor((p.z - minAABB.z()) * invLeafSize[2]));
						indices.push_back(CloudPointIndexIdx(idx, idy, idz, *it));
					}
				}
			}
		}
		std::sort(indices.begin(), indices.end(), std::less<CloudPointIndexIdx>());

		std::size_t total = 0;
		std::size_t index = 0;
		std::vector<std::pair<std::size_t, std::size_t>> firstAndLastIndices;
		firstAndLastIndices.reserve(indices.size());
		while (index < indices.size())
		{
			std::size_t i = index + 1;
			while ((i < indices.size()) && (indices[i] == indices[index]))
				++i;
			if ((i - index) >= minPointsPerVoxel)
			{
				++total;
				firstAndLastIndices.push_back(std::pair<std::size_t, std::size_t>(index, i));
			}
			index = i;
		}
		output.points.resize(total);

		//
		index = 0;
		for (std::vector<std::pair<std::size_t, std::size_t>>::const_iterator it = firstAndLastIndices.begin(); it != firstAndLastIndices.end(); ++it)
		{
			pcl::CentroidPoint<PointT> centroid;
			for (std::size_t li = it->first; li < it->second; ++li)
				centroid.add(input_->points[indices[li].cloudPointIndex]);
			centroid.get(output.points[index]);
			
			++index;
		}
	}
}
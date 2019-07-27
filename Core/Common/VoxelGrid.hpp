#pragma once

#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/common/io.h>

#include "VoxelGrid.h"

namespace RecRoom
{
	struct CloudPointIndexIdx
	{
		std::size_t idx;
		std::size_t idy;
		std::size_t idz;
		int cloudPointIndex;

		CloudPointIndexIdx(std::size_t idx, std::size_t idy, std::size_t idz, int cloudPointIndex)
			: idx(idx), idy(idy), idz(idz), cloudPointIndex(cloudPointIndex) {}
		bool operator < (const CloudPointIndexIdx& p) const
		{
			if (idx < p.idx)
				return true;
			else if (idx > p.idx)
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
			const PointT& p = (*input_)[(*it)];

			if (pcl_isfinite(p.x) &&
				pcl_isfinite(p.y) &&
				pcl_isfinite(p.z))
			{
				if ((p.x >= minAABB.x()) &&
					(p.y >= minAABB.y()) &&
					(p.z >= minAABB.z()) &&
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
		output.resize(total);

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

	template <typename PointT>
	void VNNGenerator<PointT>::Generate(std::vector<uint32_t>& cache, PcVNN& output)
	{
		if (!initCompute())
		{
			THROW_EXCEPTION("!initCompute");
			return;
		}

		std::vector<CloudPointIndexIdx> indices;
		indices.reserve(indices_->size());

		for (std::vector<int>::const_iterator it = indices_->begin(); it != indices_->end(); ++it)
		{
			const PointT& p = (*input_)[(*it)];

			if (pcl_isfinite(p.x) &&
				pcl_isfinite(p.y) &&
				pcl_isfinite(p.z))
			{
				if ((p.x >= minAABB.x()) &&
					(p.y >= minAABB.y()) &&
					(p.z >= minAABB.z()) &&
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
		cache.clear();
		output.resize(total);

		//
		index = 0;
		cache.reserve(indices.size());
		for (std::vector<std::pair<std::size_t, std::size_t>>::const_iterator it = firstAndLastIndices.begin(); it != firstAndLastIndices.end(); ++it)
		{
			float cx = 0.0f;
			float cy = 0.0f;
			float cz = 0.0f;
			std::size_t startPos = cache.size();
			for (std::size_t li = it->first; li < it->second; ++li)
			{
				int id = indices[li].cloudPointIndex;
				const PointT& pt = input_->points[id];
				cx += pt.x;
				cy += pt.y;
				cz += pt.z;
				cache.push_back(id);
			}

			PointVNN& op = output.points[index];
			op.k = cache.size() - startPos;
			if (op.k > 0)
			{
				float cs = (float)op.k;
				op.x = cx / cs;
				op.y = cy / cs;
				op.z = cz / cs;
				op.indices = &cache[startPos];
			}

			++index;
		}
	}

	template <typename PointT>
	int VNN<PointT>::nearestKSearch(
		const PointT& point, int k,
		std::vector<int>& nnIndices,
		std::vector<float>& nnSqrDist) const
	{
		THROW_EXCEPTION("Not support");
		return 0;
	}

	template <typename PointT>
	int VNN<PointT>::radiusSearch(
		const PointT& point, double radius,
		std::vector<int>& nnIndices,
		std::vector<float>& nnSqrDist,
		unsigned int maxNN = 0) const
	{
		nnIndices.clear();
		nnSqrDist.clear();

		if (maxNN != 0)
		{
			THROW_EXCEPTION("Not support")
				return 0;
		}

		std::vector<int> vnnIndices;
		std::vector<float> vnnSqrDist;
		int vnnK = accVNN->radiusSearch((PointVNN)(point), radius + diagVoxelSize, vnnIndices, vnnSqrDist);
		float vnnAcceptSqrRadius = radius - diagVoxelSize;
		vnnAcceptSqrRadius *= vnnAcceptSqrRadius;
		float sqrRadius = radius * radius;
		if (vnnK > 0)
		{
			int maxK = 0;
			for (std::vector<int>::const_iterator it = vnnIndices.begin(); it != vnnIndices.end(); ++it)
				maxK += (*pcVNN)[(*it)].k;

			nnIndices.reserve(maxK);
			nnSqrDist.reserve(maxK);

			float dx = 0.0f;
			float dy = 0.0f;
			float dz = 0.0f;
			for (int vnnI = 0; vnnI < vnnK; ++vnnI)
			{
				PointVNN& pVNN = (*pcVNN)[vnnIndices[vnnI]];
				if (vnnSqrDist[vnnI] < vnnAcceptSqrRadius)
				{
					for (int j = 0; j < pVNN.k; ++j)
					{
						int pi = pVNN.indices[j];
						const PointT& pt = (*input_)[pi];
						nnIndices.push_back(pi);

						dx = pt.x - point.x;
						dy = pt.y - point.y;
						dz = pt.z - point.z;
						nnSqrDist.push_back(dx * dx + dy * dy + dz * dz);
					}
				}
				else
				{
					for (int j = 0; j < pVNN.k; ++j)
					{
						int pi = pVNN.indices[j];
						const PointT& pt = (*input_)[pi];

						dx = pt.x - point.x;
						dy = pt.y - point.y;
						dz = pt.z - point.z;
						if ((dx * dx + dy * dy + dz * dz) < sqrRadius)
						{
							nnIndices.push_back(pi);
							nnSqrDist.push_back(dx * dx + dy * dy + dz * dz);
						}
					}
				}
			}

			return nnIndices.size();
		}
		else
			return 0;
	}
}
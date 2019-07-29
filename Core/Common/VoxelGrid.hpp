#pragma once

#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/common/io.h>

#include "VoxelGrid.h"

namespace RecRoom
{
	template <class PointType, class CenterType>
	inline bool VoxelGrid<PointType, CenterType>::GetVoxelGridIndex(const PointType& p, VoxelGridIndex& voxelGridIndex) const
	{
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
				voxelGridIndex.idx = static_cast<std::size_t> (std::floor((p.x - minAABB.x()) * invLeafSize[0]));
				voxelGridIndex.idy = static_cast<std::size_t> (std::floor((p.y - minAABB.y()) * invLeafSize[1]));
				voxelGridIndex.idz = static_cast<std::size_t> (std::floor((p.z - minAABB.z()) * invLeafSize[2]));
				return true;
			}
		}
		return false;
	}

	template <class PointType, class CenterType>
	inline void VoxelGrid<PointType, CenterType>::AddPoint(const PointType& p)
	{
		VoxelGridIndex pID;
		if (GetVoxelGridIndex(p, pID))
		{
			Leaves::iterator it = leaves->find(pID);
			if (it != leaves->end())
			{
				UpdateLeafAddPoint(it->second, p);
			}
			else
			{
				Leaf toInsert();
				InitLeaf(toInsert);
				UpdateLeafAddPoint(toInsert, p);
				leaves[pID] = toInsert;
			}
		}
	}

	template <class PointType, class CenterType>
	inline void VoxelGrid<PointType, CenterType>::DeletePoint(const PointType& p)
	{
		VoxelGridIndex pID;
		if (GetVoxelGridIndex(p, pID))
		{
			Leaves::iterator it = leaves->find(pID);
			if (it != leaves->end())
			{
				UpdateLeafDeletePoint(it->second, p);
				if (it->second.size == 0)
				{
					leaves->erase(it);
				}
			}
		}
	}

	template <class PointType, class CenterType>
	inline void VoxelGrid<PointType, CenterType>::AddPointCloud(const Pc<PointType>& pc)
	{
		for (Pc<PointType>::iterator it = pc.begin(); it != pc.end(); ++it)
			AddPoint(*it);
	}

	template <class PointType, class CenterType>
	inline void VoxelGrid<PointType, CenterType>::DeletePointCloud(const Pc<PointType>& pc)
	{
		for (Pc<PointType>::iterator it = pc.begin(); it != pc.end(); ++it)
			DeletePoint(*it);
	}

	template <class PointType>
	inline void BinaryVoxelGrid<PointType>::AddPoint(const VoxelGridIndex& pID)
	{
		Leaves::iterator it = leaves->find(pID);
		if (it != leaves->end())
		{
			it->second.size += 1;
		}
		else
		{
			Leaf toInsert();
			InitLeaf(toInsert);
			toInsert.size += 1;
			leaves[pID] = toInsert;
		}
	}

	template <class PointType>
	inline void BinaryVoxelGrid<PointType>::DeletePoint(const VoxelGridIndex& pID)
	{
		Leaves::iterator it = leaves->find(pID);
		if (it != leaves->end())
		{
			if (it->second.size == 0)
			{
				THROW_EXCEPTION("This should not be happened.");
			}

			it->second.size -= 1;

			if (it->second.size == 0)
			{
				leaves->erase(it);
			}
		}
	}

	template <class PointType>
	void BinaryVoxelGrid<PointType>::Dilation(std::size_t kernelSize, std::size_t iteration)
	{
		if ((kernelSize % 2) != 0)
			THROW_EXCEPTION("kernelSize must odd");
		std::size_t extSize = kernelSize / 2;

		if (maxIndexX < extSize)
			THROW_EXCEPTION("extSize is not valid");
		if (maxIndexY < extSize)
			THROW_EXCEPTION("extSize is not valid");
		if (maxIndexZ < extSize)
			THROW_EXCEPTION("extSize is not valid");

		for (std::size_t itr = 0; itr < iteration; ++itr)
		{
			for (Leaves::const_iterator it = leaves->begin(); it != leaves->end(); ++it)
			{
				// Iterator window
				for (std::size_t siftX = 1; (siftX <= extSize) && (it->first.idx + siftX <= maxIndexX); ++siftX)
				{
					for (std::size_t siftY = 1; (siftY <= extSize) && (it->first.idy + siftY <= maxIndexY); ++siftY)
					{
						// + + +
						for (std::size_t siftZ = 1; (siftZ <= extSize) && (it->first.idz + siftZ <= maxIndexZ); ++siftZ)
							AddPoint(VoxelGridIndex(
								it->first.idx + siftX,
								it->first.idy + siftY,
								it->first.idz + siftZ));

						// + + -
						for (std::size_t siftZ = 1; (siftZ <= extSize) && (it->first.idz >= siftZ); ++siftZ)
							AddPoint(VoxelGridIndex(
								it->first.idx + siftX,
								it->first.idy + siftY,
								it->first.idz - siftZ));
					}
					for (std::size_t siftY = 1; (siftY <= extSize) && (it->first.idy >= siftY); ++siftY)
					{
						// + - +
						for (std::size_t siftZ = 1; (siftZ <= extSize) && (it->first.idz + siftZ <= maxIndexZ); ++siftZ)
							AddPoint(VoxelGridIndex(
								it->first.idx + siftX,
								it->first.idy - siftY,
								it->first.idz + siftZ));

						// + - -
						for (std::size_t siftZ = 1; (siftZ <= extSize) && (it->first.idz >= siftZ); ++siftZ)
							AddPoint(VoxelGridIndex(
								it->first.idx + siftX,
								it->first.idy - siftY,
								it->first.idz - siftZ));
					}
				}
				for (std::size_t siftX = 1; (siftX <= extSize) && (it->first.idx >= siftX); ++siftX)
				{
					for (std::size_t siftY = 1; (siftY <= extSize) && (it->first.idy + siftY <= maxIndexY); ++siftY)
					{
						// - + +
						for (std::size_t siftZ = 1; (siftZ <= extSize) && (it->first.idz + siftZ <= maxIndexZ); ++siftZ)
							AddPoint(VoxelGridIndex(
								it->first.idx - siftX,
								it->first.idy + siftY,
								it->first.idz + siftZ));

						// - + -
						for (std::size_t siftZ = 1; (siftZ <= extSize) && (it->first.idz >= siftZ); ++siftZ)
							AddPoint(VoxelGridIndex(
								it->first.idx - siftX,
								it->first.idy + siftY,
								it->first.idz - siftZ));
					}
					for (std::size_t siftY = 1; (siftY <= extSize) && (it->first.idy >= siftY); ++siftY)
					{
						// - - +
						for (std::size_t siftZ = 1; (siftZ <= extSize) && (it->first.idz + siftZ <= maxIndexZ); ++siftZ)
							AddPoint(VoxelGridIndex(
								it->first.idx - siftX,
								it->first.idy - siftY,
								it->first.idz + siftZ));

						// - - -
						for (std::size_t siftZ = 1; (siftZ <= extSize) && (it->first.idz >= siftZ); ++siftZ)
							AddPoint(VoxelGridIndex(
								it->first.idx - siftX,
								it->first.idy - siftY,
								it->first.idz - siftZ));
					}
				}
			}
		}
	}

	template <class PointType>
	void BinaryVoxelGrid<PointType>::Erosion(std::size_t kernelSize, std::size_t iteration)
	{
		if ((kernelSize % 2) != 0)
			THROW_EXCEPTION("kernelSize must odd");
		std::size_t extSize = kernelSize / 2;

		if (maxIndexX < extSize)
			THROW_EXCEPTION("extSize is not valid");
		if (maxIndexY < extSize)
			THROW_EXCEPTION("extSize is not valid");
		if (maxIndexZ < extSize)
			THROW_EXCEPTION("extSize is not valid");

		for (std::size_t itr = 0; itr < iteration; ++itr)
		{
			for (Leaves::const_iterator it = leaves->begin(); it != leaves->end(); ++it)
			{
				// Iterator window
				for (std::size_t siftX = 1; (siftX <= extSize) && (it->first.idx + siftX <= maxIndexX); ++siftX)
				{
					for (std::size_t siftY = 1; (siftY <= extSize) && (it->first.idy + siftY <= maxIndexY); ++siftY)
					{
						// + + +
						for (std::size_t siftZ = 1; (siftZ <= extSize) && (it->first.idz + siftZ <= maxIndexZ); ++siftZ)
							DeletePoint(VoxelGridIndex(
								it->first.idx + siftX,
								it->first.idy + siftY,
								it->first.idz + siftZ));

						// + + -
						for (std::size_t siftZ = 1; (siftZ <= extSize) && (it->first.idz >= siftZ); ++siftZ)
							DeletePoint(VoxelGridIndex(
								it->first.idx + siftX,
								it->first.idy + siftY,
								it->first.idz - siftZ));
					}
					for (std::size_t siftY = 1; (siftY <= extSize) && (it->first.idy >= siftY); ++siftY)
					{
						// + - +
						for (std::size_t siftZ = 1; (siftZ <= extSize) && (it->first.idz + siftZ <= maxIndexZ); ++siftZ)
							DeletePoint(VoxelGridIndex(
								it->first.idx + siftX,
								it->first.idy - siftY,
								it->first.idz + siftZ));

						// + - -
						for (std::size_t siftZ = 1; (siftZ <= extSize) && (it->first.idz >= siftZ); ++siftZ)
							DeletePoint(VoxelGridIndex(
								it->first.idx + siftX,
								it->first.idy - siftY,
								it->first.idz - siftZ));
					}
				}
				for (std::size_t siftX = 1; (siftX <= extSize) && (it->first.idx >= siftX); ++siftX)
				{
					for (std::size_t siftY = 1; (siftY <= extSize) && (it->first.idy + siftY <= maxIndexY); ++siftY)
					{
						// - + +
						for (std::size_t siftZ = 1; (siftZ <= extSize) && (it->first.idz + siftZ <= maxIndexZ); ++siftZ)
							DeletePoint(VoxelGridIndex(
								it->first.idx - siftX,
								it->first.idy + siftY,
								it->first.idz + siftZ));

						// - + -
						for (std::size_t siftZ = 1; (siftZ <= extSize) && (it->first.idz >= siftZ); ++siftZ)
							DeletePoint(VoxelGridIndex(
								it->first.idx - siftX,
								it->first.idy + siftY,
								it->first.idz - siftZ));
					}
					for (std::size_t siftY = 1; (siftY <= extSize) && (it->first.idy >= siftY); ++siftY)
					{
						// - - +
						for (std::size_t siftZ = 1; (siftZ <= extSize) && (it->first.idz + siftZ <= maxIndexZ); ++siftZ)
							DeletePoint(VoxelGridIndex(
								it->first.idx - siftX,
								it->first.idy - siftY,
								it->first.idz + siftZ));

						// - - -
						for (std::size_t siftZ = 1; (siftZ <= extSize) && (it->first.idz >= siftZ); ++siftZ)
							DeletePoint(VoxelGridIndex(
								it->first.idx - siftX,
								it->first.idy - siftY,
								it->first.idz - siftZ));
					}
				}
			}
		}
	}

	template <class PointType>
	void BinaryVoxelGrid<PointType>::Opening(std::size_t kernelSize, std::size_t iteration)
	{
		for (std::size_t itr = 0; itr < iteration; ++itr)
		{
			Erosion(kernelSize);
			Dilation(kernelSize);
		}
	}

	template <class PointType>
	void BinaryVoxelGrid<PointType>::Closing(std::size_t kernelSize, std::size_t iteration)
	{
		for (std::size_t itr = 0; itr < iteration; ++itr)
		{
			Dilation(kernelSize);
			Erosion(kernelSize);
		}
	}

	struct IndexPair
	{
		VoxelGridIndex voxelGridIndex;
		int cloudPointIndex;

		IndexPair(const VoxelGridIndex& voxelGridIndex, int cloudPointIndex)
			: voxelGridIndex(voxelGridIndex), cloudPointIndex(cloudPointIndex) {}

		inline bool operator < (const IndexPair& p) const
		{
			return voxelGridIndex < p.voxelGridIndex;
		}

		inline bool operator == (const IndexPair& p) const
		{
			return voxelGridIndex == p.voxelGridIndex;
		}
	};

	template <typename PointType>
	void VoxelGridFilter<PointType>::applyFilter(PointCloud& output)
	{
		std::vector<IndexPair> indexPairs;
		indexPairs.reserve(indices_->size());

		for (std::vector<int>::const_iterator it = indices_->begin(); it != indices_->end(); ++it)
		{
			VoxelGridIndex voxelGridIndex;
			if (GetVoxelGridIndex((*input_)[(*it)], voxelGridIndex))
				indexPairs.push_back(IndexPair(voxelGridIndex, *it));
		}
		std::sort(indexPairs.begin(), indexPairs.end());

		std::size_t total = 0;
		std::size_t index = 0;
		std::vector<std::pair<std::size_t, std::size_t>> firstAndLastIndices;
		firstAndLastIndices.reserve(indexPairs.size());
		while (index < indexPairs.size())
		{
			std::size_t i = index + 1;
			while ((i < indexPairs.size()) && (indexPairs[i] == indexPairs[index]))
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
			pcl::CentroidPoint<PointType> centroid;
			for (std::size_t li = it->first; li < it->second; ++li)
				centroid.add(input_->points[indexPairs[li].cloudPointIndex]);
			centroid.get(output.points[index]);

			++index;
		}
	}

	template <typename PointType>
	void VNNGenerator<PointType>::Generate(std::vector<uint32_t>& cache, PcVNN& output)
	{
		if (!initCompute())
		{
			THROW_EXCEPTION("!initCompute");
			return;
		}

		std::vector<IndexPair> indexPairs;
		indexPairs.reserve(indices_->size());

		for (std::vector<int>::const_iterator it = indices_->begin(); it != indices_->end(); ++it)
		{
			VoxelGridIndex voxelGridIndex;
			if (GetVoxelGridIndex((*input_)[(*it)], voxelGridIndex))
				indexPairs.push_back(IndexPair(voxelGridIndex, *it));
		}
		std::sort(indexPairs.begin(), indexPairs.end());

		std::size_t total = 0;
		std::size_t index = 0;
		std::vector<std::pair<std::size_t, std::size_t>> firstAndLastIndices;
		firstAndLastIndices.reserve(indexPairs.size());
		while (index < indexPairs.size())
		{
			std::size_t i = index + 1;
			while ((i < indexPairs.size()) && (indexPairs[i] == indexPairs[index]))
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
		cache.reserve(indexPairs.size());
		for (std::vector<std::pair<std::size_t, std::size_t>>::const_iterator it = firstAndLastIndices.begin(); it != firstAndLastIndices.end(); ++it)
		{
			float cx = 0.0f;
			float cy = 0.0f;
			float cz = 0.0f;
			std::size_t startPos = cache.size();
			for (std::size_t li = it->first; li < it->second; ++li)
			{
				int id = indexPairs[li].cloudPointIndex;
				const PointType& pt = input_->points[id];
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

	template <typename PointType>
	int VNN<PointType>::nearestKSearch(
		const PointType& point, int k,
		std::vector<int>& nnIndices,
		std::vector<float>& nnSqrDist) const
	{
		THROW_EXCEPTION("Not support");
		return 0;
	}

	template <typename PointType>
	int VNN<PointType>::radiusSearch(
		const PointType& point, double radius,
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
						const PointType& pt = (*input_)[pi];
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
						const PointType& pt = (*input_)[pi];

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
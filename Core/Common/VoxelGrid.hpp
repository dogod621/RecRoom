#pragma once

#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/common/io.h>

#include "VoxelGrid.h"

namespace RecRoom
{
	template <class PointType, class Leaf>
	inline bool VoxelGridBase<PointType, Leaf>::PointToIndex(const PointType& p, VoxelGridIndex& voxelGridIndex) const
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

	template <class PointType, class Leaf>
	inline void VoxelGridBase<PointType, Leaf>::AddPointCloud(CONST_PTR(Pc<PointType>) input, CONST_PTR(PcIndex) filter)
	{
		if (filter)
		{
			for (PcIndex::const_iterator it = filter->begin(); it != filter->end(); ++it)
				AddPoint((*input)[(*it)]);
		}
		else
		{
			for (Pc<PointType>::const_iterator it = input->begin(); it != input->end(); ++it)
				AddPoint(*it);
		}
	}

	template <class PointType, class Leaf>
	inline void VoxelGridBase<PointType, Leaf>::DeletePointCloud(CONST_PTR(Pc<PointType>) input, CONST_PTR(PcIndex) filter)
	{
		if (filter)
		{
			for (PcIndex::const_iterator it = filter->begin(); it != filter->end(); ++it)
				DeletePoint((*input)[(*it)]);
		}
		else
		{
			for (Pc<PointType>::const_iterator it = input->begin(); it != input->end(); ++it)
				DeletePoint(*it);
		}
	}

	template <class PointType>
	inline void VoxelGrid<PointType>::AddPoint(const PointType& p)
	{
		VoxelGridIndex pID;
		if (PointToIndex(p, pID))
		{
			Leaves::iterator it = leaves->find(pID);
			if (it != leaves->end())
			{
				UpdateLeafAddPoint(it->second, p);
			}
			else
			{
				Leaf toInsert;
				InitLeaf(toInsert);
				UpdateLeafAddPoint(toInsert, p);
				(*leaves)[pID] = toInsert;
			}
		}
	}

	template <class PointType>
	inline void VoxelGrid<PointType>::DeletePoint(const PointType& p)
	{
		VoxelGridIndex pID;
		if (PointToIndex(p, pID))
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


	template <class PointType>
	inline void BinaryVoxelGrid<PointType>::AddPoint(const PointType& p)
	{
		VoxelGridIndex pID;
		if (PointToIndex(p, pID))
		{
			(*leaves)[pID] = true;
		}
	}

	template <class PointType>
	inline void BinaryVoxelGrid<PointType>::DeletePoint(const PointType& p)
	{
		VoxelGridIndex pID;
		if (PointToIndex(p, pID))
		{
			Leaves::iterator it = leaves->find(pID);
			if (it != leaves->end())
				leaves->erase(it);
		}
	}

	template <class PointType>
	inline void BinaryVoxelGrid<PointType>::AddPoint(const VoxelGridIndex& pID)
	{
		(*leaves)[pID] = true;
	}

	template <class PointType>
	inline void BinaryVoxelGrid<PointType>::DeletePoint(const VoxelGridIndex& pID)
	{
		Leaves::iterator it = leaves->find(pID);
		if (it != leaves->end())
			leaves->erase(it);
	}

	template <class PointType>
	inline void BinaryVoxelGrid<PointType>::AddPointCloud(CONST_PTR(Pc<PointType>) input, CONST_PTR(PcIndex) filter)
	{
		std::vector<IndexPair> indexPairs;
		if (filter)
		{
			indexPairs.reserve(filter->size());
			for (std::vector<int>::const_iterator it = filter->begin(); it != filter->end(); ++it)
			{
				VoxelGridIndex voxelGridIndex;
				if (PointToIndex((*input)[(*it)], voxelGridIndex))
					indexPairs.push_back(IndexPair(voxelGridIndex, *it));
			}
		}
		else
		{
			indexPairs.reserve(input->size());
			for (int px = 0; px < input->size(); ++px)
			{
				VoxelGridIndex voxelGridIndex;
				if (PointToIndex((*input)[px], voxelGridIndex))
					indexPairs.push_back(IndexPair(voxelGridIndex, px));
			}
		}
		std::sort(indexPairs.begin(), indexPairs.end());

		std::size_t index = 0;
		while (index < indexPairs.size())
		{
			std::size_t i = index + 1;
			while ((i < indexPairs.size()) && (indexPairs[i] == indexPairs[index]))
				++i;
			AddPoint(indexPairs[index].voxelGridIndex);
			index = i;
		}
	}

	template <class PointType>
	inline void BinaryVoxelGrid<PointType>::DeletePointCloud(CONST_PTR(Pc<PointType>) input, CONST_PTR(PcIndex) filter)
	{
		std::vector<IndexPair> indexPairs;
		if (filter)
		{
			indexPairs.reserve(filter->size());
			for (std::vector<int>::const_iterator it = filter->begin(); it != filter->end(); ++it)
			{
				VoxelGridIndex voxelGridIndex;
				if (PointToIndex((*input)[(*it)], voxelGridIndex))
					indexPairs.push_back(IndexPair(voxelGridIndex, *it));
			}
		}
		else
		{
			indexPairs.reserve(input->size());
			for (int px = 0; px < input->size(); ++px)
			{
				VoxelGridIndex voxelGridIndex;
				if (PointToIndex((*input)[px], voxelGridIndex))
					indexPairs.push_back(IndexPair(voxelGridIndex, px));
			}
		}
		std::sort(indexPairs.begin(), indexPairs.end());

		std::size_t index = 0;
		while (index < indexPairs.size())
		{
			std::size_t i = index + 1;
			while ((i < indexPairs.size()) && (indexPairs[i] == indexPairs[index]))
				++i;
			DeletePoint(indexPairs[index].voxelGridIndex);
			index = i;
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
		Erosion(kernelSize, iteration);
		Dilation(kernelSize, iteration);
	}

	template <class PointType>
	void BinaryVoxelGrid<PointType>::Closing(std::size_t kernelSize, std::size_t iteration)
	{
		Dilation(kernelSize, iteration);
		Erosion(kernelSize, iteration);
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
			if (PointToIndex((*input_)[(*it)], voxelGridIndex))
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
				centroid.add((*input_).points[indexPairs[li].cloudPointIndex]);
			centroid.get(output[index]);

			++index;
		}
	}

	template <typename PointType>
	void VNN<PointType>::setInputCloud(const PointCloudConstPtr& cloud, const IndicesConstPtr& indices = nullptr)
	{
		pcVNN->clear();
		cache.clear();
		
		PRINT_INFO("Build VNN - Start");

		std::vector<IndexPair> indexPairs;
		if (indices)
		{
			indexPairs.reserve(indices->size());
			for (std::vector<int>::const_iterator it = indices->begin(); it != indices->end(); ++it)
			{
				VoxelGridIndex voxelGridIndex;
				if (PointToIndex((*cloud)[(*it)], voxelGridIndex))
					indexPairs.push_back(IndexPair(voxelGridIndex, *it));
			}
		}
		else
		{
			indexPairs.reserve(cloud->size());
			for (int px = 0; px < cloud->size(); ++px)
			{
				VoxelGridIndex voxelGridIndex;
				if (PointToIndex((*cloud)[px], voxelGridIndex))
					indexPairs.push_back(IndexPair(voxelGridIndex, px));
			}
		}
		std::sort(indexPairs.begin(), indexPairs.end());

		std::size_t total = 0;
		std::size_t index = 0;
		pcVNN->reserve(indexPairs.size());
		cache.reserve(indexPairs.size());
		while (index < indexPairs.size())
		{
			std::size_t i = index + 1;
			while ((i < indexPairs.size()) && (indexPairs[i] == indexPairs[index]))
				++i;
			if ((i - index) >= minPointsPerVoxel)
			{
				++total;
				PointVNN op = PointVNN(IndexToPoint(indexPairs[index].voxelGridIndex));
				std::size_t startPos = cache.size();
				for (std::size_t j = index; j < i; ++j)
					cache.push_back(indexPairs[j].cloudPointIndex;);
				op.k = cache.size() - startPos;
				op.indices = &cache[startPos];
				pcVNN->push_back(op);
			}
			index = i;
		}
		std::stringstream ss;
		ss << "Build VNN - End - vnnSize: " << pcVNN->size() << ", cacheSize:" << cache.size();

		PRINT_INFO(ss.str());

		PRINT_INFO("Build AccVNN - Start");
		accVNN->setInputCloud(pcVNN);
		PRINT_INFO("Build AccVNN - End");

		input_ = cloud;
		indices_ = indices;
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
#pragma once

#include <pcl/filters/boost.h>
#include <pcl/filters/filter.h>
#include <pcl/search/search.h>

#include <map>

#include "Common.h"
#include "Point.h"

namespace RecRoom
{
	struct VoxelGridIndex
	{
		std::size_t idx;
		std::size_t idy;
		std::size_t idz;

		VoxelGridIndex(std::size_t idx = 0, std::size_t idy = 0, std::size_t idz = 0)
			: idx(idx), idy(idy), idz(idz)
		{}

		inline bool operator < (const VoxelGridIndex& p) const
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

		inline bool operator == (const VoxelGridIndex& p) const
		{
			return (idx == p.idx) && (idy == p.idy) && (idz == p.idz);
		}
	};

	template <class CenterType>
	struct VoxelGridLeaf
	{
		CenterType center;
		std::size_t size;

		VoxelGridLeaf(std::size_t size = 0)
			: size(size)
		{}

		VoxelGridLeaf(CenterType center, std::size_t size = 0)
			: center(center), size(size)
		{}
	};

	template <class PointType, class CenterType>
	class VoxelGrid
	{
	public:
		using Leaf = VoxelGridLeaf<CenterType>;
		using Leaves = std::map<VoxelGridIndex, VoxelGridLeaf<CenterType>>;

	public:
		VoxelGrid(const Eigen::Vector3d& leafSize, const Eigen::Vector3d& minAABB, const Eigen::Vector3d& maxAABB)
			: leafSize(leafSize), minAABB(minAABB), maxAABB(maxAABB), leaves(new Leaves)
		{
			if (!(leafSize.x() > 0.0))
				THROW_EXCEPTION("!(leafSize.x() > 0.0)");
			if (!(leafSize.y() > 0.0))
				THROW_EXCEPTION("!(leafSize.y() > 0.0)");
			if (!(leafSize.z() > 0.0))
				THROW_EXCEPTION("!(leafSize.z() > 0.0)");
			invLeafSize = Eigen::Array3d::Ones() / leafSize.array();

			if (std::floor((maxAABB.x() - minAABB.x()) * invLeafSize[0]) > static_cast<double>(std::numeric_limits<std::size_t>::max()))
				THROW_EXCEPTION("X dimention overflow");
			if (std::floor((maxAABB.y() - minAABB.y()) * invLeafSize[1]) > static_cast<double>(std::numeric_limits<std::size_t>::max()))
				THROW_EXCEPTION("Y dimention overflow");
			if (std::floor((maxAABB.z() - minAABB.z()) * invLeafSize[2]) > static_cast<double>(std::numeric_limits<std::size_t>::max()))
				THROW_EXCEPTION("Z dimention overflow");

			maxIndexX = static_cast<std::size_t> (std::floor((maxAABB.x() - minAABB.x()) * invLeafSize[0]));
			maxIndexY = static_cast<std::size_t> (std::floor((maxAABB.y() - minAABB.y()) * invLeafSize[1]));
			maxIndexZ = static_cast<std::size_t> (std::floor((maxAABB.z() - minAABB.z()) * invLeafSize[2]));
		}

		inline bool GetVoxelGridIndex(const PointType& p, VoxelGridIndex& voxelGridIndex) const;

		inline std::size_t Size() const
		{
			return leaves->size();
		}

		inline void AddPoint(const PointType& p);

		inline void DeletePoint(const PointType& p);

		inline void AddPointCloud(const Pc<PointType>& pc);

		inline void DeletePointCloud(const Pc<PointType>& pc);

		inline virtual PTR(Pc<PointType>) GetPointCloud() const
		{
			THROW_EXCEPTION("Interafce is not implement");
		}

	protected:
		inline virtual void InitLeaf(Leaf& leaf) const 
		{ 
			{ // Implement here 
				THROW_EXCEPTION("Interafce is not implement");
			}
			leaf.size = 0; 
		}
		inline virtual void UpdateLeafAddPoint(Leaf& leaf, const PointType& point)
		{
			{ // Implement here 
				THROW_EXCEPTION("Interafce is not implement");
			}
			leaf.size += 1;
		}
		inline virtual void UpdateLeafDeletePoint(Leaf& leaf, const PointType& point)
		{
			if (leaf.size == 0)
			{
				THROW_EXCEPTION("This should not be happened.");
			}
			{ // Implement here 
				THROW_EXCEPTION("Interafce is not implement");
			}
			leaf.size -= 1;
		}

	protected:
		Eigen::Vector3d leafSize;
		Eigen::Array3d invLeafSize;
		Eigen::Vector3d minAABB, maxAABB;
		std::size_t maxIndexX;
		std::size_t maxIndexY;
		std::size_t maxIndexZ;
		PTR(Leaves) leaves;
		
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};

	template <class PointType>
	class BinaryVoxelGrid : public VoxelGrid<PointType, bool>
	{
	public:
		using Leaf = VoxelGridLeaf<bool>;
		using Leaves = std::vector<Leaf>;

	public:
		BinaryVoxelGrid(const Eigen::Vector3d& leafSize, const Eigen::Vector3d& minAABB, const Eigen::Vector3d& maxAABB)
			: VoxelGrid(leafSize, minAABB, maxAABB)
		{
		}

		inline virtual PTR(Pc<PointType>) GetPointCloud() const
		{
			PTR(Pc<PointType>) pc(new Pc<PointType>);
			pc->reserve(leaves->size());
			for (Leaves::const_iterator it = leaves->begin(); it != leaves->end(); ++it)
			{
				PointType p;
				p.x = ((float)(it->first.idx) + 0.5f) * (float)(leafSize)+minAABB.x();
				p.y = ((float)(it->first.idy) + 0.5f) * (float)(leafSize)+minAABB.y();
				p.z = ((float)(it->first.idz) + 0.5f) * (float)(leafSize)+minAABB.z();
				pc->push_back(p);
			}
			return pc;
		}

		inline void AddPoint(const VoxelGridIndex& pID);

		inline void DeletePoint(const VoxelGridIndex& pID);

		void Dilation(std::size_t kernelSize = 1, std::size_t iteration = 1);

		void Erosion(std::size_t kernelSize = 1, std::size_t iteration = 1);

		void Opening(std::size_t kernelSize = 1, std::size_t iteration = 1);

		void Closing(std::size_t kernelSize = 1, std::size_t iteration = 1);

	protected:
		inline virtual void InitLeaf(Leaf& leaf) const
		{
			leaf.size = 0;
		}
		inline virtual void UpdateLeafAddPoint(Leaf& leaf, const PointType& point)
		{
			leaf.size += 1;
		}

		inline virtual void UpdateLeafDeletePoint(Leaf& leaf, const PointType& point)
		{
			if (leaf.size == 0)
			{
				THROW_EXCEPTION("This should not be happened.");
			}
			leaf.size -= 1;
		}

	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};

	template <class PointType>
	class VoxelGridFilter : public pcl::Filter<PointType>, public VoxelGrid<PointType, bool>
	{
	public:
		using Leaf = VoxelGridLeaf<bool>;
		using Leaves = std::vector<Leaf>;

	protected:
		using pcl::Filter<PointType>::filter_name_;
		using pcl::Filter<PointType>::input_;
		using pcl::Filter<PointType>::indices_;

		using PointCloud = pcl::Filter<PointType>::PointCloud;

	public:
		VoxelGridFilter(const Eigen::Vector3d& leafSize, const Eigen::Vector3d& minAABB, const Eigen::Vector3d& maxAABB, std::size_t minPointsPerVoxel = 0)
			: pcl::Filter<PointType>(), VoxelGrid<PointType, bool>(leafSize, minAABB, maxAABB), minPointsPerVoxel(minPointsPerVoxel)
		{
			filter_name_ = "VoxelGridFilter";
		}

	protected:
		std::size_t minPointsPerVoxel;

		void applyFilter(PointCloud& output);

	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};

	template <class PointType>
	class VNNGenerator : public VoxelGridFilter<PointType>
	{
	public:
		VNNGenerator(const Eigen::Vector3d& leafSize, const Eigen::Vector3d& minAABB, const Eigen::Vector3d& maxAABB)
			: VoxelGridFilter<PointType>(leafSize, minAABB, maxAABB)
		{}

		void Generate(std::vector<uint32_t>& cache, PcVNN& output);
	};

	template <class PointType>
	class VNN : public pcl::search::Search<PointType>
	{
	public:
		typedef typename pcl::search::Search<PointType>::PointCloud PointCloud;
		typedef typename pcl::search::Search<PointType>::PointCloudConstPtr PointCloudConstPtr;

		typedef boost::shared_ptr<std::vector<int> > IndicesPtr;
		typedef boost::shared_ptr<const std::vector<int> > IndicesConstPtr;

	public:
		VNN(const Eigen::Vector3d& leafSize, const Eigen::Vector3d& minAABB, const Eigen::Vector3d& maxAABB) 
			: pcl::search::Search<PointType>("VNN", false), cache(), pcVNN(new PcVNN), accVNN(new KDTreeVNN), vnnGen(leafSize, minAABB, maxAABB)
		{
			diagVoxelSize = std::sqrt(
				leafSize.x() * leafSize.x()+ 
				leafSize.y() * leafSize.y()+ 
				leafSize.z() * leafSize.z());
		}

		void setSortedResults(bool sorted_results) { THROW_EXCEPTION("Not support"); }

		void setInputCloud(const PointCloudConstPtr& cloud, const IndicesConstPtr& indices = nullptr)
		{
			PRINT_INFO("Build VNN - Start");
			vnnGen.setInputCloud(cloud);
			if(indices)
				vnnGen.setIndices(indices);
			vnnGen.Generate(cache, *pcVNN);
			std::stringstream ss;
			ss << "Build VNN - End - vnnSize: " << pcVNN->size() << ", cacheSize:" << cache.size();
			PRINT_INFO(ss.str());

			PRINT_INFO("Build AccVNN - Start");
			accVNN->setInputCloud(pcVNN);
			PRINT_INFO("Build AccVNN - End");

			input_ = cloud;
			indices_ = indices;
		}

		int nearestKSearch(
			const PointType& point, int k,
			std::vector<int>& nnIndices,
			std::vector<float>& nnSqrDist) const;

		int radiusSearch(
			const PointType& point, double radius,
			std::vector<int>& nnIndices,
			std::vector<float>& nnSqrDist,
			unsigned int maxNN = 0) const;

	protected:
		double diagVoxelSize;
		PTR(PcVNN) pcVNN;
		PTR(AccVNN) accVNN;
		std::vector<uint32_t> cache;
		VNNGenerator<PointType> vnnGen;
	};
}

#include "VoxelGrid.hpp"
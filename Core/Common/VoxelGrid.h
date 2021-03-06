#pragma once

#include <pcl/filters/boost.h>
#include <pcl/filters/filter.h>
#include <pcl/search/search.h>

#include <map>

#include "Common.h"
#include "Point.h"

namespace RecRoom
{
	enum MorphologyOperation : Flag
	{
		MorphologyOperation_NONE = 0,
		DILATION = 1,
		EROSION = 2,
		OPENING = 3,
		CLOSING = 4
	};

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

	template <class PointType, class LeafType>
	class VoxelGridBase
	{
	public:
		using Leaf = LeafType;
		using Leaves = std::map<const VoxelGridIndex, Leaf>;

	public:
		VoxelGridBase(const Eigen::Vector3d& leafSize, const Eigen::Vector3d& minAABB, const Eigen::Vector3d& maxAABB)
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

		inline bool PointToIndex(const PointType& p, VoxelGridIndex& voxelGridIndex) const;

		inline std::size_t Size() const
		{
			return leaves->size();
		}

		inline virtual void AddPoint(const PointType& p)
		{
			THROW_EXCEPTION("Interafce is not implement");
		}

		inline virtual void DeletePoint(const PointType& p)
		{
			THROW_EXCEPTION("Interafce is not implement");
		}

		inline virtual void AddPointCloud(CONST_PTR(Pc<PointType>) input, CONST_PTR(PcIndex) filter = nullptr);

		inline virtual void DeletePointCloud(CONST_PTR(Pc<PointType>) input, CONST_PTR(PcIndex) filter = nullptr);

		inline virtual PTR(Pc<PointType>) GetPointCloud() const
		{
			THROW_EXCEPTION("Interafce is not implement");
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
	struct VoxelGridLeaf
	{
		PointType center;
		std::size_t size;

		VoxelGridLeaf(std::size_t size = 0)
			: size(size)
		{}

		VoxelGridLeaf(PointType center, std::size_t size = 0)
			: center(center), size(size)
		{}
	};

	template <class PointType>
	class VoxelGrid : public VoxelGridBase<PointType, VoxelGridLeaf<PointType>>
	{
	public:
		using Leaf = VoxelGridBase<PointType, VoxelGridLeaf<PointType>>::Leaf;
		using Leaves = VoxelGridBase<PointType, VoxelGridLeaf<PointType>>::Leaves;

	public:
		VoxelGrid(const Eigen::Vector3d& leafSize, const Eigen::Vector3d& minAABB, const Eigen::Vector3d& maxAABB)
			: VoxelGridBase<PointType, VoxelGridLeaf<PointType>>(leafSize, minAABB, maxAABB)
		{}

		inline virtual void AddPoint(const PointType& p);

		inline virtual void DeletePoint(const PointType& p);

		inline virtual PTR(Pc<PointType>) GetPointCloud() const
		{
			PTR(Pc<PointType>) pc(new Pc<PointType>);
			pc->reserve(leaves->size());
			for (Leaves::const_iterator it = leaves->begin(); it != leaves->end(); ++it)
				pc->push_back(it->second.center);
			return pc;
		}

	protected:
		inline virtual void InitLeaf(Leaf& leaf) const
		{
			{
				THROW_EXCEPTION("Interafce is not implement"); // implement here
			}
			leaf.size = 0;
		}
		inline virtual void UpdateLeafAddPoint(Leaf& leaf, const PointType& point)
		{
			{
				THROW_EXCEPTION("Interafce is not implement"); // implement here
			}
			leaf.size += 1;
		}
		inline virtual void UpdateLeafDeletePoint(Leaf& leaf, const PointType& point)
		{
			if (leaf.size == 0)
			{
				THROW_EXCEPTION("This should not be happened.");
			}
			{
				THROW_EXCEPTION("Interafce is not implement"); // implement here
			}
			leaf.size -= 1;
		}

	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};

	template <class PointType>
	class BinaryVoxelGrid : public VoxelGridBase<PointType, bool>, public ThreadAble
	{
	protected:
		static void MarkBoundaryTask(
			int id,
			void* self,
			void* its);

	public:
		using Leaf = VoxelGridBase<PointType, bool>::Leaf;
		using Leaves = VoxelGridBase<PointType, bool>::Leaves;

	public:
		BinaryVoxelGrid(const Eigen::Vector3d& leafSize, const Eigen::Vector3d& minAABB, const Eigen::Vector3d& maxAABB)
			: VoxelGridBase(leafSize, minAABB, maxAABB), ThreadAble()
		{
		}

		inline PointType IndexToPoint(const VoxelGridIndex& index) const
		{
			PointType p;
			p.x = ((float)(index.idx) + 0.5f) * (float)(leafSize.x()) + minAABB.x();
			p.y = ((float)(index.idy) + 0.5f) * (float)(leafSize.y()) + minAABB.y();
			p.z = ((float)(index.idz) + 0.5f) * (float)(leafSize.z()) + minAABB.z();
			return p;
		}

		inline virtual PTR(Pc<PointType>) GetPointCloud() const
		{
			PTR(Pc<PointType>) pc(new Pc<PointType>);
			pc->reserve(leaves->size());
			for (Leaves::const_iterator it = leaves->begin(); it != leaves->end(); ++it)
				pc->push_back(IndexToPoint(it->first));
			return pc;
		}

		inline virtual void AddPoint(const PointType& p);

		inline virtual void DeletePoint(const PointType& p);

		inline void AddPoint(const VoxelGridIndex& pID);

		inline void DeletePoint(const VoxelGridIndex& pID);

		inline virtual void AddPointCloud(CONST_PTR(Pc<PointType>) input, CONST_PTR(PcIndex) filter = nullptr);

		inline virtual void DeletePointCloud(CONST_PTR(Pc<PointType>) input, CONST_PTR(PcIndex) filter = nullptr);

	public:
		void MarkBoundary();

		void Dilation(std::size_t kernelSize = 1, std::size_t iteration = 1);

		void Erosion(std::size_t kernelSize = 1, std::size_t iteration = 1);

		void Opening(std::size_t kernelSize = 1, std::size_t iteration = 1);

		void Closing(std::size_t kernelSize = 1, std::size_t iteration = 1);

	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};

	template <class PointType>
	class VoxelGridFilter : public pcl::Filter<PointType>, public VoxelGridBase<PointType, bool>
	{
	public:
		using Leaf = VoxelGridBase<PointType, bool>::Leaf;
		using Leaves = VoxelGridBase<PointType, bool>::Leaves;

	protected:
		using pcl::Filter<PointType>::filter_name_;
		using pcl::Filter<PointType>::input_;
		using pcl::Filter<PointType>::indices_;

		using PointCloud = pcl::Filter<PointType>::PointCloud;

	public:
		VoxelGridFilter(const Eigen::Vector3d& leafSize, const Eigen::Vector3d& minAABB, const Eigen::Vector3d& maxAABB, std::size_t minPointsPerVoxel = 0)
			: pcl::Filter<PointType>(), VoxelGridBase<PointType, bool>(leafSize, minAABB, maxAABB), minPointsPerVoxel(minPointsPerVoxel)
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
	class VNN : public pcl::search::Search<PointType>, public BinaryVoxelGrid<PointType>
	{
	public:
		typedef typename pcl::search::Search<PointType>::PointCloud PointCloud;
		typedef typename pcl::search::Search<PointType>::PointCloudConstPtr PointCloudConstPtr;

		typedef boost::shared_ptr<std::vector<int> > IndicesPtr;
		typedef boost::shared_ptr<const std::vector<int> > IndicesConstPtr;

	public:
		VNN(const Eigen::Vector3d& leafSize, const Eigen::Vector3d& minAABB, const Eigen::Vector3d& maxAABB) 
			: pcl::search::Search<PointType>("VNN", false), cache(), pcVNN(new PcVNN), accVNN(new KDTreeVNN), BinaryVoxelGrid<PointType>(leafSize, minAABB, maxAABB)
		{
			diagVoxelSize = std::sqrt(
				leafSize.x() * leafSize.x()+ 
				leafSize.y() * leafSize.y()+ 
				leafSize.z() * leafSize.z());
		}

		void setSortedResults(bool sorted_results) { THROW_EXCEPTION("Not support"); }

		void setInputCloud(const PointCloudConstPtr& cloud, const IndicesConstPtr& indices = nullptr);

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
	};
}

#include "VoxelGrid.hpp"
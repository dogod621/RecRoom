#pragma once

#include <pcl/filters/boost.h>
#include <pcl/filters/filter.h>
#include <pcl/search/search.h>

#include <map>

#include "Common.h"
#include "Point.h"

namespace RecRoom
{
	template <typename PointT>
	class VoxelGridFilter : public pcl::Filter<PointT>
	{
	protected:
		using pcl::Filter<PointT>::filter_name_;
		using pcl::Filter<PointT>::input_;
		using pcl::Filter<PointT>::indices_;

		using PointCloud = pcl::Filter<PointT>::PointCloud;

	public:

		VoxelGridFilter(const Eigen::Vector3d& leafSize, const Eigen::Vector3d& minAABB, const Eigen::Vector3d& maxAABB) 
			: leafSize(leafSize),
			minAABB(minAABB),
			maxAABB(maxAABB),
			minPointsPerVoxel(0)
		{
			filter_name_ = "VoxelGridFilter";

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
		}

	protected:
		Eigen::Vector3d leafSize;
		Eigen::Array3d invLeafSize;
		Eigen::Vector3d minAABB, maxAABB;

		/** \brief Minimum number of points per voxel for the centroid to be computed */
		std::size_t minPointsPerVoxel;

		void applyFilter(PointCloud& output);

	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};

	template <typename PointT>
	class VNNGenerator : public VoxelGridFilter<PointT>
	{
	public:

		VNNGenerator(const Eigen::Vector3d& leafSize, const Eigen::Vector3d& minAABB, const Eigen::Vector3d& maxAABB) 
			: VoxelGridFilter(leafSize, minAABB, maxAABB)
		{}

		void Generate(std::vector<uint32_t>& cache, PcVNN& output);
	};

	template <typename PointT>
	class VNN : public pcl::search::Search<PointT>
	{
	public:
		typedef typename pcl::search::Search<PointT>::PointCloud PointCloud;
		typedef typename pcl::search::Search<PointT>::PointCloudConstPtr PointCloudConstPtr;

		typedef boost::shared_ptr<std::vector<int> > IndicesPtr;
		typedef boost::shared_ptr<const std::vector<int> > IndicesConstPtr;

	public:

		VNN(const Eigen::Vector3d& leafSize, const Eigen::Vector3d& minAABB, const Eigen::Vector3d& maxAABB) 
			: pcl::search::Search<PointT>("VNN", false), cache(), pcVNN(new PcVNN), accVNN(new KDTreeVNN), vnnGen(leafSize, minAABB, maxAABB)
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
			const PointT& point, int k,
			std::vector<int>& nnIndices,
			std::vector<float>& nnSqrDist) const;

		int radiusSearch(
			const PointT& point, double radius,
			std::vector<int>& nnIndices,
			std::vector<float>& nnSqrDist,
			unsigned int maxNN = 0) const;

	protected:
		double diagVoxelSize;
		PTR(PcVNN) pcVNN;
		PTR(AccVNN) accVNN;
		std::vector<uint32_t> cache;
		VNNGenerator<PointT> vnnGen;
	};
}

#include "VoxelGrid.hpp"
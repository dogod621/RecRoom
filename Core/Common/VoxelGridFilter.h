#pragma once

#include <pcl/filters/boost.h>
#include <pcl/filters/filter.h>
#include <map>

#include "Common.h"

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

		VoxelGridFilter(const Eigen::Vector3d& leafSize, const Eigen::Vector3d& minAABB, const Eigen::Vector3d& maxAABB) :
			leafSize(leafSize),
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
	};
}

#include "VoxelGridFilter.hpp"
#pragma once

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/filter_indices.h>

#include "Common.h"

namespace RecRoom
{
	template<typename PointT>
	class CropAABB : public pcl::FilterIndices<PointT>
	{
		typedef typename pcl::Filter<PointT>::PointCloud PointCloud;
		typedef typename PointCloud::Ptr PointCloudPtr;
		typedef typename PointCloud::ConstPtr PointCloudConstPtr;

	public:

		typedef boost::shared_ptr< CropAABB<PointT> > Ptr;
		typedef boost::shared_ptr< const CropAABB<PointT> > ConstPtr;

		CropAABB(
			const Eigen::Vector3f& minAABB,
			const Eigen::Vector3f& maxAABB) :
			pcl::FilterIndices<PointT>::FilterIndices(false),
			minAABB(minAABB),
			maxAABB(maxAABB)
		{
			filter_name_ = "CropAABB";
		}

	protected:
		using pcl::PCLBase<PointT>::input_;
		using pcl::PCLBase<PointT>::indices_;
		using pcl::Filter<PointT>::filter_name_;
		using pcl::FilterIndices<PointT>::negative_;
		using pcl::FilterIndices<PointT>::keep_organized_;
		using pcl::FilterIndices<PointT>::user_filter_value_;
		using pcl::FilterIndices<PointT>::removed_indices_;
		using FilterIndices<PointT>::extract_removed_indices_;

		void applyFilter(PointCloud& output);

		void applyFilter(PcIndex& indices);

	private:
		Eigen::Vector3f minAABB;
		Eigen::Vector3f maxAABB;
	};
}

#include "CropAABB.hpp"
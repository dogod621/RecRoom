#pragma once

#include <pcl/outofcore/outofcore.h>
#include <pcl/outofcore/outofcore_impl.h>

#include "Common.h"
#include "Data.h"

namespace RecRoom
{
	template<class PointType>
	class Container : public Data, public pcl::outofcore::OutofcoreOctreeBase<pcl::outofcore::OutofcoreOctreeDiskContainer<PointType>, PointType>
	{
	public:
		using Self = Container<PointType>;
		using Ptr = PTR(Self);
		using ConstPtr = CONST_PTR(Self);

		Container(const boost::filesystem::path &root_node_name, const bool load_all)
			: Data(), pcl::outofcore::OutofcoreOctreeBase<pcl::outofcore::OutofcoreOctreeDiskContainer<PointType>, PointType>(root_node_name, load_all) {}
		Container(const Eigen::Vector3d& min, const Eigen::Vector3d& max, const double resolution_arg, const boost::filesystem::path &root_node_name, const std::string &coord_sys)
			: Data(), pcl::outofcore::OutofcoreOctreeBase<pcl::outofcore::OutofcoreOctreeDiskContainer<PointType>, PointType>(min, max, resolution_arg, root_node_name, coord_sys) {}
		Container(const boost::uint64_t max_depth, const Eigen::Vector3d &min, const Eigen::Vector3d &max, const boost::filesystem::path &root_node_name, const std::string &coord_sys)
			: Data(), pcl::outofcore::OutofcoreOctreeBase<pcl::outofcore::OutofcoreOctreeDiskContainer<PointType>, PointType>(max_depth, min, max, root_node_name, coord_sys) {}
	};
}

#include "Container.hpp"
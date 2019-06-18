#pragma once

#include "E57Reconstructor.h"

namespace RecRoom
{
	E57Reconstructor::E57Reconstructor(const boost::filesystem::path& rawContainerPath, const Eigen::Vector3d& min, const Eigen::Vector3d& max, const double resolution)
		: rawContainerPath(rawContainerPath)
	{
		if (!boost::filesystem::exists(rawContainerPath))
		{
			boost::filesystem::create_directory(rawContainerPath);
			PRINT_INFO("Create directory: " + rawContainerPath.string());
		}
		rawContainer = RAWContainer::Ptr(new RAWContainer(min, max, resolution,
			rawContainerPath / boost::filesystem::path("RAW") / boost::filesystem::path("root.oct_idx"), "ECEF"));
		ndfContainer = NDFContainer::Ptr(new NDFContainer(Eigen::Vector3d(-1.0, -1.0, -1.0), Eigen::Vector3d((double)std::numeric_limits<unsigned int>::max(), 1.0, 1.0), 1.0,
			rawContainerPath / boost::filesystem::path("NDF") / boost::filesystem::path("root.oct_idx"), "ECEF"));
	}

	E57Reconstructor::E57Reconstructor(const boost::filesystem::path& rawContainerPath)
		: rawContainerPath(rawContainerPath)
	{
		rawContainer = RAWContainer::Ptr(new RAWContainer(rawContainerPath / boost::filesystem::path("RAW") / boost::filesystem::path("root.oct_idx"), true));
		ndfContainer = NDFContainer::Ptr(new NDFContainer(rawContainerPath / boost::filesystem::path("NDF") / boost::filesystem::path("root.oct_idx"), true));
	}
}
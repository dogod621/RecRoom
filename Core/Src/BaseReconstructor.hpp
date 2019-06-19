#pragma once

#include "StringUtils.h"
#include "BaseReconstructor.h"
#include "AsyncProcess.h"

namespace RecRoom
{
	template<class RawPointType_, class MedPointType_, class RecPointType_>
	Reconstructor<RawPointType_, MedPointType_, RecPointType_>::Reconstructor(const boost::filesystem::path& filePath, const Eigen::Vector3d& min, const Eigen::Vector3d& max, const double resolution, const double outofCoreLeafOverlap_)
		: Base(), filePath(filePath)
	{
		if (!boost::filesystem::exists(filePath))
		{
			boost::filesystem::create_directory(filePath);
			PRINT_INFO("Create directory: " + filePath.string());
		}

		rawContainer = RAWContainerPtr(new RAWContainer(min, max, resolution,
			filePath / boost::filesystem::path("RAW") / boost::filesystem::path("root.oct_idx"), "ECEF"));
		ndfContainer = NDFContainerPtr(new NDFContainer(16, Eigen::Vector3d(-1.0, -1.0, -1.0), Eigen::Vector3d((double)std::numeric_limits<unsigned short>::max(), 1.0, 1.0),
			filePath / boost::filesystem::path("NDF") / boost::filesystem::path("root.oct_idx"), "ECEF"));

		if (outofCoreLeafOverlap_ < 0)
			outofCoreLeafOverlap = rawContainer->getVoxelSideLength() / 16;
	}

	template<class RawPointType_, class MedPointType_, class RecPointType_>
	Reconstructor<RawPointType_, MedPointType_, RecPointType_>::Reconstructor(const boost::filesystem::path& filePath, const double outofCoreLeafOverlap_)
		: Base(), filePath(filePath)
	{
		if (!IsFileE57Rec(filePath, true))
			THROW_EXCEPTION("filePath is not valid.");

		rawContainer = RAWContainerPtr(new RAWContainer(filePath / boost::filesystem::path("RAW") / boost::filesystem::path("root.oct_idx"), true));
		ndfContainer = NDFContainerPtr(new NDFContainer(filePath / boost::filesystem::path("NDF") / boost::filesystem::path("root.oct_idx"), true));

		if (outofCoreLeafOverlap_ < 0)
			outofCoreLeafOverlap = rawContainer->getVoxelSideLength() / 16;
	}

	template<class RawPointType_, class MedPointType_, class RecPointType_>
	void Reconstructor<RawPointType_, MedPointType_, RecPointType_>::ReconstructLOD(const double samplePercent)
	{
		rawContainer->setSamplePercent(samplePercent);
		rawContainer->buildLOD();
	}
}
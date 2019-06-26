#pragma once

#include "ScannerPcE57.h"

namespace RecRoom
{
	class ScannerPcBLK360 : public ScannerPcE57, public DumpAble
	{
	public:
		ScannerPcBLK360(
			const boost::filesystem::path& filePath,
			const boost::filesystem::path& e57FilePath,
			const boost::filesystem::path& lfFilePath,
			const PTR(ContainerPcRAW)& containerPcRAW,
			unsigned char colorThresh = 6);

	public:
		virtual bool Valid(const PointRAW& pointRAW) const
		{
#ifdef POINT_RAW_WITH_RGB
			return pcl::isFinite(pointRAW) && ((pointRAW.r > colorThresh) | (pointRAW.g > colorThresh) | (pointRAW.b > colorThresh));// Use to remove black noise which is caused by BLK360
#else
			return pcl::isFinite(pointRAW);
#endif
		}

	public:
		unsigned char getColorThresh() const { return colorThresh; }

	protected:
		unsigned char colorThresh; // Use to remove black noise which is caused by BLK360

		virtual void Load() { DumpAble::Load(); };
		virtual void Dump() const { DumpAble::Dump(); };
		virtual void Load(const nlohmann::json& j);
		virtual void Dump(nlohmann::json& j) const;
		virtual bool CheckExist() const { return DumpAble::CheckExist(); };
	};
}

#include "ScannerPcBLK360.hpp"
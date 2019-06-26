#pragma once

#include "ScannerPcE57.h"

namespace RecRoom
{
	class ScannerPcBLK360 : public ScannerPcE57
	{
	public:
		ScannerPcBLK360(
			const boost::filesystem::path& filePath,
			const boost::filesystem::path& e57FilePath,
			const boost::filesystem::path& lfFilePath,
			const PTR(ContainerPcRAW)& containerPcRAW,
			unsigned char colorThresh = 6);

	public:
		virtual void ShipPcLFData() const { THROW_EXCEPTION("NOT DONE"); }
		virtual bool Valid(const PointRAW& pointRAW) const
		{
#ifdef POINT_RAW_WITH_RGB
			return pcl::isFinite(pointRAW) && ((pointRAW.r > colorThresh) | (pointRAW.g > colorThresh) | (pointRAW.b > colorThresh));// Use to remove black noise which is caused by BLK360
#else
			return pcl::isFinite(pointRAW);
#endif
		}

	public:
		boost::filesystem::path getFilePath() const { return filePath; }
		unsigned char getColorThresh() const { return colorThresh; }

	protected:
		boost::filesystem::path filePath;
		unsigned char colorThresh; // Use to remove black noise which is caused by BLK360

		virtual void LoadMeta();
		virtual void DumpMeta() const;
	};
}

#include "ScannerPcBLK360.hpp"
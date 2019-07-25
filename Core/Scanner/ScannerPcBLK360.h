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
		virtual std::size_t ScanImageWidth() const { return 8000; };
		virtual std::size_t ScanImageHeight() const { return 4000; };
		virtual Eigen::Vector3d ToScanImageUVDepth(const Eigen::Vector3d& worldXYZ) const
		{
			Eigen::Vector3d rae = CoodConvert<CoordSys::RAE_PE_PX_PY, CoordSys::XYZ_PX_PY_PZ>(worldXYZ);
			Eigen::Vector2d uv = ToUV(ToMapping(UVMode::PANORAMA, CoordSys::RAE_PE_PX_PY), rae);
			return Eigen::Vector3d(uv.x(), uv.y(), rae.x());
		};

		virtual bool ToScanLaser(const PointRAW& scanPoint, ScanLaser& scanLaser) const;

	protected:
		virtual bool Valid(const PointRAW& pointRAW) const
		{
#ifdef INPUT_PERPOINT_RGB
			return pcl::isFinite(pointRAW) && ((pointRAW.r >= colorThresh) | (pointRAW.g >= colorThresh) | (pointRAW.b >= colorThresh));// Use to remove black noise which is caused by BLK360
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
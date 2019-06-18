#pragma once

#include <memory>

#include "E57Format.h"

#include "Common.h"
#include "Coordinate.h"
#include "PointType.h"
#include "Base.h"

namespace RecRoom
{
	std::ostream& operator << (std::ostream& os, const e57::ImageFile& v);

	template<Scanner scanner>
	class E57ScanMeta : public ScanData<PointE57, scanner>
	{
	public:
		E57ScanMeta() : ScanData() {}

		virtual nlohmann::json DumpToJson();
		virtual void LoadFromJson(const nlohmann::json& j);
		
	protected:
		bool hasPointXYZ;
		bool hasPointRGB;
		bool hasPointI;
	};

	template<Scanner scanner>
	class E57ScanData : public E57ScanMeta<scanner>
	{
	public:
		E57ScanData() : E57ScanMeta(), coodSys(CoodSys::CoodSys_UNKNOWN){}

		virtual void Load(const e57::ImageFile& imf, const e57::VectorNode& data3D, int64_t serialNumber);

		virtual void ToPointCloud(pcl::PointCloud<PointE57>& scanCloud);

		operator E57ScanMeta() const
		{
			E57ScanMeta meta;
			meta.hasPointXYZ = hasPointXYZ;
			meta.hasPointRGB = hasPointRGB;
			meta.hasPointI = hasPointI;
			meta.numPoints = numPoints;
			meta.numValidPoints = numValidPoints;
			meta.serialNumber = serialNumber;
			return meta;
		}

	protected:
		CoodSys coodSys;
		std::shared_ptr<float> x;
		std::shared_ptr<float> y;
		std::shared_ptr<float> z;
		std::shared_ptr<float> i;
		std::shared_ptr<uint8_t> r;
		std::shared_ptr<uint8_t> g;
		std::shared_ptr<uint8_t> b;
	};
}
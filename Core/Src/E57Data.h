#pragma once

#include <vector>

#include "E57Format.h"

#include "Common.h"
#include "Coordinate.h"
#include "PointType.h"
#include "BaseData.h"

namespace RecRoom
{
	class E57ScanData : public ScanData<PointE57>
	{
	public:
		using Ptr = boost::shared_ptr<E57ScanData>;
		using ConstPtr = boost::shared_ptr<const E57ScanData>;

	public:
		E57ScanData() : ScanData(), coodSys(CoodSys::CoodSys_UNKNOWN), hasPointXYZ(false), hasPointRGB(false), hasPointI(false) {}
		void ClearBuffers();

	public:
		virtual void FromJson(const nlohmann::json& j);
		virtual void ToJson(nlohmann::json& j) const;

		virtual void FromPointCloud(const pcl::PointCloud<PointE57>& scanCloud) { THROW_EXCEPTION("Interface is not implemented"); }
		virtual void ToPointCloud(pcl::PointCloud<PointE57>& scanCloud) const;

		virtual void FromE57Format(const e57::ImageFile& imf, const e57::VectorNode& data3D, int64_t serialNumber);
		virtual void ToE57Format(const e57::ImageFile& imf, const e57::VectorNode& data3D, int64_t serialNumber) const { THROW_EXCEPTION("Interface is not implemented"); }

	protected:
		CoodSys coodSys;
		bool hasPointXYZ;
		bool hasPointRGB;
		bool hasPointI;
		std::vector<float> xBuffer;
		std::vector<float> yBuffer;
		std::vector<float> zBuffer;
		std::vector<float> iBuffer;
		std::vector<uint8_t> rBuffer;
		std::vector<uint8_t> gBuffer;
		std::vector<uint8_t> bBuffer;
	};

	//
	void OStreamE57NodeFormat(std::ostream& os, std::size_t pDepth, const e57::Node& pNode);
	void OStreamE57NodeFormat(std::ostream& os, std::size_t pDepth, const e57::StructureNode& pNode);
	void OStreamE57NodeFormat(std::ostream& os, std::size_t pDepth, const e57::VectorNode& pNode);
	void OStreamE57NodeFormat(std::ostream& os, std::size_t pDepth, const e57::CompressedVectorNode& pNode);
	void OStreamE57NodeFormat(std::ostream& os, std::size_t pDepth, const e57::IntegerNode& pNode);
	void OStreamE57NodeFormat(std::ostream& os, std::size_t pDepth, const e57::ScaledIntegerNode& pNode);
	void OStreamE57NodeFormat(std::ostream& os, std::size_t pDepth, const e57::FloatNode& pNode);
	void OStreamE57NodeFormat(std::ostream& os, std::size_t pDepth, const e57::StringNode& pNode);
	void OStreamE57NodeFormat(std::ostream& os, std::size_t pDepth, const e57::BlobNode& pNode);

	std::ostream& operator << (std::ostream& os, const e57::ImageFile& v);
}

#include "E57Data.hpp"
#include "nlohmann/json.hpp"

#include "ContainerPcNDFOCT.h"

namespace RecRoom
{
	ContainerPcNDFOCT::ContainerPcNDFOCT(const boost::filesystem::path& filePath_)
		: filePath(filePath_), ContainerPcNDF(), oct(nullptr), numSegment(0)
	{
		if (!boost::filesystem::is_directory(filePath))
			THROW_EXCEPTION("filePath is not valid.");
		if (!boost::filesystem::is_directory(filePath / boost::filesystem::path("NDF")))
			THROW_EXCEPTION("filePath is not valid: missing ./NDF/");
		if (!boost::filesystem::exists(filePath / boost::filesystem::path("NDF") / boost::filesystem::path("root.oct_idx")))
			THROW_EXCEPTION("filePath is not valid: missing ./NDF/root.oct_idx.");
		if (!boost::filesystem::exists(filePath / boost::filesystem::path("metaNDF.txt")))
			THROW_EXCEPTION("filePath is not valid: missing ./metaNDF.txt.");

		oct = OCT::Ptr(new OCT(filePath / boost::filesystem::path("NDF") / boost::filesystem::path("root.oct_idx"), true));

		std::string metaPath = (filePath / boost::filesystem::path("metaNDF.txt")).string();
		std::ifstream file(metaPath, std::ios_base::in);
		if (!file)
			throw pcl::PCLException("Load file " + metaPath + " failed.");
		nlohmann::json j;
		file >> j;

		if (j.find("numSegment") == j.end())
			THROW_EXCEPTION("metaNDF is not valid: missing \"numSegment\"");
		numSegment = j["numSegment"];

		file.close();
	}

	ContainerPcNDFOCT::ContainerPcNDFOCT(const boost::filesystem::path& filePath_, std::size_t numSegment)
		: filePath(filePath_), ContainerPcNDF(), oct(nullptr), numSegment(numSegment)
	{
		if (numSegment >= std::numeric_limits<unsigned short>::max())
			THROW_EXCEPTION("numSegment is too large, max: " + std::to_string(std::numeric_limits<unsigned short>::max()));

		if (!boost::filesystem::exists(filePath))
		{
			boost::filesystem::create_directory(filePath);
			PRINT_INFO("Create directory: " + filePath.string());
		}

		oct = OCT::Ptr(new OCT(
			16,
			Eigen::Vector3d(-1.0, -1.0, -1.0), 
			Eigen::Vector3d((double)std::numeric_limits<unsigned short>::max(), 1.0, 1.0), 
			filePath / boost::filesystem::path("NDF") / boost::filesystem::path("root.oct_idx"), "ECEF"));

		std::string metaPath = (filePath / boost::filesystem::path("metaNDF.txt")).string();
		std::ofstream file(metaPath, std::ios_base::out);
		if (!file)
			throw pcl::PCLException("Create file " + metaPath + " failed.");
		nlohmann::json j;
		j["numSegment"] = numSegment;
		file << j;
		file.close();
	}

	PTR(PcNDF) ContainerPcNDFOCT::Quary(std::size_t i) const
	{
		if (i >= std::numeric_limits<unsigned short>::max())
			THROW_EXCEPTION("i is too large, max: " + std::to_string(std::numeric_limits<unsigned short>::max()));


		PTR(PcNDF) q (new PcNDF);

		//
		pcl::PCLPointCloud2::Ptr blob(new pcl::PCLPointCloud2);
		oct->queryBoundingBox(
			Eigen::Vector3d((double)i - 0.5, -1.0, -1.0),
			Eigen::Vector3d((double)i + 0.5, 1.0, 1.0),
			16, blob);
		pcl::fromPCLPointCloud2(*blob, *q);

		return q;
	}
}
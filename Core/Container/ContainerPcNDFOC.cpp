#include "nlohmann/json.hpp"

#include "ContainerPcNDFOC.h"

namespace RecRoom
{
	ContainerPcNDFOC::ContainerPcNDFOC(const boost::filesystem::path& filePath_)
		: filePath(filePath_), ContainerPcNDF(), oct(nullptr)
	{
		bool createNew = false;
		if (!boost::filesystem::is_directory(filePath))
		{
			createNew = true;
		}
		else if (!boost::filesystem::is_directory(filePath / boost::filesystem::path("NDF")))
		{
			createNew = true;
			PRINT_WARNING("filePath is not valid: missing ./NDF/, create new");
		}
		else if (!boost::filesystem::exists(filePath / boost::filesystem::path("NDF") / boost::filesystem::path("root.oct_idx")))
		{
			createNew = true;
			PRINT_WARNING("filePath is not valid: missing ./NDF/root.oct_idx, create new");
		}
		else if(!boost::filesystem::exists(filePath / boost::filesystem::path("metaNDF.txt")))
		{
			createNew = true;
			PRINT_WARNING("filePath is not valid: missing ./metaNDF.txt, create new");
		}

		if (createNew)
		{
			if (!boost::filesystem::exists(filePath))
			{
				boost::filesystem::create_directory(filePath);
				PRINT_INFO("Create directory: " + filePath.string());
			}

			oct = OCT::Ptr(new OCT(
				16,
				Eigen::Vector3d(0, 0, 0),
				Eigen::Vector3d((double)std::numeric_limits<unsigned short>::max(), 1.0, 1.0),
				filePath / boost::filesystem::path("NDF") / boost::filesystem::path("root.oct_idx"), "ECEF"));

			DumpMeta();
		}
		else
		{
			oct = OCT::Ptr(new OCT(filePath / boost::filesystem::path("NDF") / boost::filesystem::path("root.oct_idx"), true));

			LoadMeta();
		}

		//
		if(!oct)
			THROW_EXCEPTION("oct is not created?")
	}

	PTR(PcNDF) ContainerPcNDFOC::Quary(std::size_t i) const
	{
		if (i >= std::numeric_limits<unsigned short>::max())
			THROW_EXCEPTION("i is too large, max: " + std::to_string(std::numeric_limits<unsigned short>::max()));


		PTR(PcNDF) q (new PcNDF);

		//
		pcl::PCLPointCloud2::Ptr blob(new pcl::PCLPointCloud2);
		oct->queryBoundingBox(
			Eigen::Vector3d((double)i, 0.0, 0.0),
			Eigen::Vector3d((double)(i + 1), 1.0, 1.0),
			16, blob);
		pcl::fromPCLPointCloud2(*blob, *q);

		return q;
	}

	void ContainerPcNDFOC::LoadMeta()
	{
		std::string metaPath = (filePath / boost::filesystem::path("metaNDF.txt")).string();
		std::ifstream file(metaPath, std::ios_base::in);
		if (!file)
			THROW_EXCEPTION("Load file " + metaPath + " failed.");
		nlohmann::json j;
		file >> j;

		//

		//
		file.close();
	}

	void ContainerPcNDFOC::DumpMeta() const
	{
		std::string metaPath = (filePath / boost::filesystem::path("metaNDF.txt")).string();
		std::ofstream file(metaPath, std::ios_base::out);
		if (!file)
			THROW_EXCEPTION("Create file " + metaPath + " failed.");
		nlohmann::json j;

		//

		//
		file << j;
		file.close();
	}
}
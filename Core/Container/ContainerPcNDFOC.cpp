#include "nlohmann/json.hpp"

#include "ContainerPcNDFOC.h"

namespace RecRoom
{
	ContainerPcNDFOC::ContainerPcNDFOC(const boost::filesystem::path& filePath_)
		: filePath(filePath_), ContainerPcNDF(), oct(nullptr), size(0)
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
				Eigen::Vector3d(
					(double)std::numeric_limits<unsigned short>::max(), 
					(double)std::numeric_limits<unsigned short>::max(),
					(double)std::numeric_limits<unsigned short>::max()),
				filePath / boost::filesystem::path("NDF") / boost::filesystem::path("root.oct_idx"), "ECEF"));

			if(oct->getTreeDepth() != 16)
				THROW_EXCEPTION("treeDepth is not 16")
			if (std::abs(oct->getVoxelSideLength() - 1.0) > Common::eps)
				THROW_EXCEPTION("voxelSideLength is not 1")

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

	void ContainerPcNDFOC::Merge(const PTR(PcNDF)& v)
	{
		oct->addPointCloud(v);

		// update 
		int maxSegID = -1;
		OCT::Iterator it(*oct);
		while (*it != nullptr)
		{
			if ((*it)->getNodeType() == pcl::octree::LEAF_NODE)
			{
				Eigen::Vector3d minAABB;
				Eigen::Vector3d maxAABB;
				(*it)->getBoundingBox(minAABB, maxAABB);
				Eigen::Vector3d center = (maxAABB + minAABB) * 0.5;
				int segID = std::floor(center[0]);
				if ((segID < 0) || (segID >= std::numeric_limits<unsigned short>::max()))
					THROW_EXCEPTION("segID is not valid");
				if (segID > maxSegID)
					maxSegID = segID;
			}
			it++;
		}
		size = maxSegID + 1;

		//
		DumpMeta();
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
		if (j.find("size") == j.end())
			THROW_EXCEPTION("metaNDF is not valid: missing \"size\"");
		size = j["size"];

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
		j["size"] = size;

		//
		file << j;
		file.close();
	}
}
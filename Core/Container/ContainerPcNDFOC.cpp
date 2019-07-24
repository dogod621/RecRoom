#include "ContainerPcNDFOC.h"

namespace RecRoom
{
	ContainerPcNDFOC::ContainerPcNDFOC(const boost::filesystem::path& filePath_)
		: DumpAble("ContainerPcNDFOC", filePath_), ContainerPcNDF(), oct(nullptr), size(0)
	{
		if (CheckExist())
		{
			oct = OCT::Ptr(new OCT(filePath / boost::filesystem::path("pcNDF") / boost::filesystem::path("root.oct_idx"), true));
			Load();
		}
		else
		{
			oct = OCT::Ptr(new OCT(
				16,
				Eigen::Vector3d(0, 0, 0),
				Eigen::Vector3d(
				(double)std::numeric_limits<unsigned short>::max()+1,
					(double)std::numeric_limits<unsigned short>::max()+1,
					(double)std::numeric_limits<unsigned short>::max()+1),
				filePath / boost::filesystem::path("pcNDF") / boost::filesystem::path("root.oct_idx"), "ECEF"));

			if (oct->getTreeDepth() != 16)
				THROW_EXCEPTION("treeDepth is not 16");
			if (std::abs(oct->getVoxelSideLength() - 1.0) > Common::eps)
				THROW_EXCEPTION("voxelSideLength is not 1: " + std::to_string(oct->getVoxelSideLength()));

			Dump();
		}

		//
		if (!oct)
			THROW_EXCEPTION("oct is not created?");
	}

	void ContainerPcNDFOC::Merge(const CONST_PTR(PcNDF)& v)
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
				if ((segID < 0) || (segID > std::numeric_limits<unsigned short>::max()))
					THROW_EXCEPTION("segID is not valid");
				if (segID > maxSegID)
					maxSegID = segID;
			}
			it++;
		}
		size = maxSegID + 1;

		//
		Dump();
	}

	ContainerPcNDFOC::Data ContainerPcNDFOC::GetData(std::size_t i) const
	{
		if (i > std::numeric_limits<unsigned short>::max())
			THROW_EXCEPTION("i is too large, max: " + std::to_string(std::numeric_limits<unsigned short>::max()));


		Data q (new PcNDF);

		//
		pcl::PCLPointCloud2::Ptr blob(new pcl::PCLPointCloud2);
		oct->queryBoundingBox(
			Eigen::Vector3d((double)i, 0.0, 0.0),
			Eigen::Vector3d((double)(i + 1), 1.0, 1.0),
			16, blob);
		pcl::fromPCLPointCloud2(*blob, *q);

		return q;
	}

	void ContainerPcNDFOC::Load(const nlohmann::json& j)
	{
		if (j.find("size") == j.end())
			THROW_EXCEPTION("File is not valid: missing \"size\"");
		size = j["size"];
	}

	void ContainerPcNDFOC::Dump(nlohmann::json& j) const
	{
		j["size"] = size;
	}

	bool ContainerPcNDFOC::CheckExist() const
	{
		if (!DumpAble::CheckExist())
			return false;
		if (!boost::filesystem::is_directory(filePath / boost::filesystem::path("pcNDF")))
			return false;
		if (!boost::filesystem::exists(filePath / boost::filesystem::path("pcNDF") / boost::filesystem::path("root.oct_idx")))
			return false;
		return true;
	}
}
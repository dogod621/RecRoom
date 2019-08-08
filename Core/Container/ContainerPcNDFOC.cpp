#include "ContainerPcNDFOC.h"

namespace RecRoom
{
	ContainerPcNDFOC::ContainerPcNDFOC(const boost::filesystem::path& filePath_)
		: DumpAble("ContainerPcNDFOC", filePath_), ContainerPcNDF(), oct(nullptr), numLabel(0), numSerialNumber(0)
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
			if (std::abs(oct->getVoxelSideLength() - 1.0) > std::numeric_limits<float>::epsilon())
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
		int maxScanID = -1;
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

				int scanID = std::floor(center[1]);
				if ((scanID < 0) || (scanID > std::numeric_limits<unsigned short>::max()))
					THROW_EXCEPTION("scanID is not valid");
				if (scanID > maxScanID)
					maxScanID = scanID;
			}
			it++;
		}
		numLabel = maxSegID + 1;
		numSerialNumber = maxScanID + 1;

		//
		Dump();
	}

	ContainerPcNDFOC::Data ContainerPcNDFOC::GetData(std::size_t label, std::size_t serialNumber) const
	{
		if (label > std::numeric_limits<unsigned short>::max())
			THROW_EXCEPTION("label is too large, max: " + std::to_string(std::numeric_limits<unsigned short>::max()));
		if (serialNumber > std::numeric_limits<unsigned short>::max())
			THROW_EXCEPTION("serialNumber is too large, max: " + std::to_string(std::numeric_limits<unsigned short>::max()));


		Data q (new PcNDF);

		//
		pcl::PCLPointCloud2::Ptr blob(new pcl::PCLPointCloud2);
		oct->queryBoundingBox(
			Eigen::Vector3d((double)label, (double)serialNumber, 0.0),
			Eigen::Vector3d((double)(label + 1),
				(double)(serialNumber + 1),
				(double)std::numeric_limits<unsigned short>::max() + 1),
			16, blob);
		pcl::fromPCLPointCloud2(*blob, *q);

		return q;
	}

	ContainerPcNDFOC::Data ContainerPcNDFOC::GetData(std::size_t label) const
	{
		if (label > std::numeric_limits<unsigned short>::max())
			THROW_EXCEPTION("label is too large, max: " + std::to_string(std::numeric_limits<unsigned short>::max()));

		Data q(new PcNDF);

		//
		pcl::PCLPointCloud2::Ptr blob(new pcl::PCLPointCloud2);
		oct->queryBoundingBox(
			Eigen::Vector3d((double)label, 0.0, 0.0),
			Eigen::Vector3d((double)(label + 1),
				(double)std::numeric_limits<unsigned short>::max() + 1,
				(double)std::numeric_limits<unsigned short>::max() + 1),
			16, blob);
		pcl::fromPCLPointCloud2(*blob, *q);

		return q;
	}

	void ContainerPcNDFOC::Load(const nlohmann::json& j)
	{
		if (j.find("numLabel") == j.end())
			THROW_EXCEPTION("File is not valid: missing \"numLabel\"");
		numLabel = j["numLabel"];

		if (j.find("numSerialNumber") == j.end())
			THROW_EXCEPTION("File is not valid: missing \"numSerialNumber\"");
		numSerialNumber = j["numSerialNumber"];
	}

	void ContainerPcNDFOC::Dump(nlohmann::json& j) const
	{
		j["numLabel"] = numLabel;

		j["numSerialNumber"] = numSerialNumber;
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
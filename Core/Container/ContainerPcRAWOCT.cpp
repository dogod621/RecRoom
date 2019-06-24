#include <fstream>

#include <pcl/filters/crop_box.h>

#include "nlohmann/json.hpp"

#include "ContainerPcRAWOCT.h"

namespace RecRoom
{
	ContainerPcRAWOCT::ContainerPcRAWOCT(const boost::filesystem::path& filePath_)
		: filePath(filePath_), ContainerPcRAW(), oct(nullptr), outOfCoreOverlapSize(0.0), quaries()
	{
		if (!boost::filesystem::is_directory(filePath))
			THROW_EXCEPTION("filePath is not valid.");
		if (!boost::filesystem::is_directory(filePath / boost::filesystem::path("RAW")))
			THROW_EXCEPTION("filePath is not valid: missing ./RAW/");
		if (!boost::filesystem::exists(filePath / boost::filesystem::path("RAW") / boost::filesystem::path("root.oct_idx")))
			THROW_EXCEPTION("filePath is not valid: missing ./RAW/root.oct_idx.");
		if (!boost::filesystem::exists(filePath / boost::filesystem::path("metaRAW.txt")))
			THROW_EXCEPTION("filePath is not valid: missing ./metaRAW.txt.");

		oct = OCT::Ptr(new OCT(filePath / boost::filesystem::path("RAW") / boost::filesystem::path("root.oct_idx"), true));

		std::string metaPath = (filePath / boost::filesystem::path("metaRAW.txt")).string();
		std::ifstream file(metaPath, std::ios_base::in);
		if (!file)
			THROW_EXCEPTION("Load file " + metaPath + " failed.");
		nlohmann::json j;
		file >> j;

		for (nlohmann::json::const_iterator qit = j["quaries"].begin(); qit != j["quaries"].end(); ++qit)
		{
			QuaryPcRAWOCT q;
			{
				if (qit->find("minAABB") == qit->end())
					THROW_EXCEPTION("metaRAW is not valid: missing \"minAABB\"");
				nlohmann::json::const_iterator it = (*qit)["minAABB"].begin();
				q.minAABB.x() = *it; ++it;
				q.minAABB.y() = *it; ++it;
				q.minAABB.z() = *it; ++it;
			}

			{
				if (qit->find("maxAABB") == qit->end())
					THROW_EXCEPTION("metaRAW is not valid: missing \"maxAABB\"");
				nlohmann::json::const_iterator it = (*qit)["maxAABB"].begin();
				q.maxAABB.x() = *it; ++it;
				q.maxAABB.y() = *it; ++it;
				q.maxAABB.z() = *it; ++it;
			}

			{
				if (qit->find("extMinAABB") == qit->end())
					THROW_EXCEPTION("metaRAW is not valid: missing \"extMinAABB\"");
				nlohmann::json::const_iterator it = (*qit)["extMinAABB"].begin();
				q.extMinAABB.x() = *it; ++it;
				q.extMinAABB.y() = *it; ++it;
				q.extMinAABB.z() = *it; ++it;
			}

			{
				if (qit->find("extMaxAABB") == qit->end())
					THROW_EXCEPTION("metaRAW is not valid: missing \"extMaxAABB\"");
				nlohmann::json::const_iterator it = (*qit)["extMaxAABB"].begin();
				q.extMaxAABB.x() = *it; ++it;
				q.extMaxAABB.y() = *it; ++it;
				q.extMaxAABB.z() = *it; ++it;
			}

			if (qit->find("depth") == qit->end())
				THROW_EXCEPTION("metaRAW is not valid: missing \"depth\"");
			q.depth = (*qit)["depth"];

			quaries.push_back(q);
		}

		if (j.find("outOfCoreOverlapSize") == j.end())
			THROW_EXCEPTION("metaRAW is not valid: missing \"outOfCoreOverlapSize\"");
		outOfCoreOverlapSize = j["outOfCoreOverlapSize"];

		file.close();

		//
		if (!oct)
			THROW_EXCEPTION("oct is not created?")
	}

	ContainerPcRAWOCT::ContainerPcRAWOCT(const boost::filesystem::path& filePath_, const Eigen::Vector3d& min, const Eigen::Vector3d& max, const double res, double outOfCoreOverlapSize)
		: filePath(filePath_), ContainerPcRAW(), oct(nullptr), outOfCoreOverlapSize(outOfCoreOverlapSize), quaries()
	{
		if (!boost::filesystem::exists(filePath))
		{
			boost::filesystem::create_directory(filePath);
			PRINT_INFO("Create directory: " + filePath.string());
		}

		oct = OCT::Ptr(new OCT(min, max, res, filePath / boost::filesystem::path("RAW") / boost::filesystem::path("root.oct_idx"), "ECEF"));

		std::string metaPath = (filePath / boost::filesystem::path("metaRAW.txt")).string();
		std::ofstream file(metaPath, std::ios_base::out);
		if (!file)
			THROW_EXCEPTION("Create file " + metaPath + " failed.");
		nlohmann::json j;
		j["outOfCoreOverlapSize"] = outOfCoreOverlapSize;
		file << j;
		file.close();

		//
		if (!oct)
			THROW_EXCEPTION("oct is not created?")
	}

	void ContainerPcRAWOCT::Merge(const PTR(PcRAW)& v)
	{
		oct->addPointCloud(v);

		// update 
		quaries.clear();
		OCT::Iterator it(*oct);
		Eigen::Vector3d ext(outOfCoreOverlapSize, outOfCoreOverlapSize, outOfCoreOverlapSize);
		while (*it != nullptr)
		{
			if ((*it)->getNodeType() == pcl::octree::LEAF_NODE)
			{
				Eigen::Vector3d minAABB;
				Eigen::Vector3d maxAABB;
				std::size_t depth;
				(*it)->getBoundingBox(minAABB, maxAABB);
				quaries.push_back(QuaryPcRAWOCT(minAABB, maxAABB, minAABB - ext, maxAABB + ext, (*it)->getDepth()));
			}
			it++;
		}

		//
		std::string metaPath = (filePath / boost::filesystem::path("metaRAW.txt")).string();
		std::ofstream file(metaPath, std::ios_base::out);
		if (!file)
			THROW_EXCEPTION("Create file " + metaPath + " failed.");
		nlohmann::json j;
		j["outOfCoreOverlapSize"] = outOfCoreOverlapSize;

		for (std::size_t i = 0; i < quaries.size(); ++i)
		{
			nlohmann::json jQ;

			jQ["minAABB"].push_back(quaries[i].minAABB.x());
			jQ["minAABB"].push_back(quaries[i].minAABB.y());
			jQ["minAABB"].push_back(quaries[i].minAABB.z());

			jQ["maxAABB"].push_back(quaries[i].maxAABB.x());
			jQ["maxAABB"].push_back(quaries[i].maxAABB.y());
			jQ["maxAABB"].push_back(quaries[i].maxAABB.z());

			jQ["extMinAABB"].push_back(quaries[i].extMinAABB.x());
			jQ["extMinAABB"].push_back(quaries[i].extMinAABB.y());
			jQ["extMinAABB"].push_back(quaries[i].extMinAABB.z());

			jQ["extMaxAABB"].push_back(quaries[i].extMaxAABB.x());
			jQ["extMaxAABB"].push_back(quaries[i].extMaxAABB.y());
			jQ["extMaxAABB"].push_back(quaries[i].extMaxAABB.z());

			jQ["depth"] = quaries[i].depth;
		}

		file << j;
		file.close();
	}

	QuaryPcRAW ContainerPcRAWOCT::Quary(std::size_t i) const
	{
		if (i >= quaries.size())
			THROW_EXCEPTION("i is too large, max: " + std::to_string(quaries.size()));


		QuaryPcRAW q;

		//
		pcl::PCLPointCloud2::Ptr blob(new pcl::PCLPointCloud2);
		oct->queryBoundingBox(quaries[i].extMinAABB, quaries[i].extMaxAABB, quaries[i].depth, blob);
		pcl::fromPCLPointCloud2(*blob, *q.data);

		//
		pcl::CropBox<PointRAW> cb;
		cb.setMin(Eigen::Vector4f(quaries[i].minAABB.x(), quaries[i].minAABB.y(), quaries[i].minAABB.z(), 1.0));
		cb.setMax(Eigen::Vector4f(quaries[i].maxAABB.x(), quaries[i].maxAABB.y(), quaries[i].maxAABB.z(), 1.0));
		cb.setInputCloud(q.data);
		cb.filter(*q.index);

		return q;
	}
}
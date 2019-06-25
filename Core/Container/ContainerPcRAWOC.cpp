#include <fstream>

#include <pcl/filters/crop_box.h>

#include "nlohmann/json.hpp"

#include "ContainerPcRAWOC.h"

namespace RecRoom
{
	ContainerPcRAWOC::ContainerPcRAWOC(const boost::filesystem::path& filePath_, 
		const Eigen::Vector3d& min, const Eigen::Vector3d& max, const double res, double overlap)
		: filePath(filePath_), ContainerPcRAW(), oct(nullptr), overlap(overlap), quaries()
	{
		bool createNew = false;
		if (!boost::filesystem::is_directory(filePath))
		{
			createNew = true;
		}
		else if (!boost::filesystem::is_directory(filePath / boost::filesystem::path("RAW")))
		{
			createNew = true;
			PRINT_WARNING("filePath is not valid: missing ./RAW/, create new");
		}
		else if (!boost::filesystem::exists(filePath / boost::filesystem::path("RAW") / boost::filesystem::path("root.oct_idx")))
		{
			createNew = true;
			PRINT_WARNING("filePath is not valid: missing ./RAW/root.oct_idx, create new");
		}
		else if (!boost::filesystem::exists(filePath / boost::filesystem::path("metaRAW.txt")))
		{
			createNew = true;
			PRINT_WARNING("filePath is not valid: missing ./metaRAW.txt, create new");
		}

		if (createNew)
		{
			if (!boost::filesystem::exists(filePath))
			{
				boost::filesystem::create_directory(filePath);
				PRINT_INFO("Create directory: " + filePath.string());
			}

			oct = OCT::Ptr(new OCT(min, max, res, filePath / boost::filesystem::path("RAW") / boost::filesystem::path("root.oct_idx"), "ECEF"));

			DumpMeta();
		}
		else
		{
			oct = OCT::Ptr(new OCT(filePath / boost::filesystem::path("RAW") / boost::filesystem::path("root.oct_idx"), true));

			LoadMeta();
		}

		//
		if (!oct)
			THROW_EXCEPTION("oct is not created?")
	}

	void ContainerPcRAWOC::Merge(const PTR(PcRAW)& v)
	{
		oct->addPointCloud(v);

		// update 
		quaries.clear();
		OCT::Iterator it(*oct);
		Eigen::Vector3d ext(overlap, overlap, overlap);
		while (*it != nullptr)
		{
			if ((*it)->getNodeType() == pcl::octree::LEAF_NODE)
			{
				Eigen::Vector3d minAABB;
				Eigen::Vector3d maxAABB;
				std::size_t depth;
				(*it)->getBoundingBox(minAABB, maxAABB);
				quaries.push_back(QuaryMeta(minAABB, maxAABB, minAABB - ext, maxAABB + ext, (*it)->getDepth()));
			}
			it++;
		}

		//
		DumpMeta();
	}

	ContainerPcRAW::QuaryData ContainerPcRAWOC::Quary(std::size_t i) const
	{
		if (i >= quaries.size())
			THROW_EXCEPTION("i is too large, max: " + std::to_string(quaries.size()));

		ContainerPcRAW::QuaryData q (quaries[i]);

		//
		pcl::PCLPointCloud2::Ptr blob(new pcl::PCLPointCloud2);
		oct->queryBoundingBox(quaries[i].extMinAABB, quaries[i].extMaxAABB, quaries[i].depth, blob);
		pcl::fromPCLPointCloud2(*blob, *q.data);

		//
		pcl::CropBox<PointMED> cb;
		cb.setMin(Eigen::Vector4f(quaries[i].minAABB.x(), quaries[i].minAABB.y(), quaries[i].minAABB.z(), 1.0));
		cb.setMax(Eigen::Vector4f(quaries[i].maxAABB.x(), quaries[i].maxAABB.y(), quaries[i].maxAABB.z(), 1.0));
		cb.setInputCloud(q.data);
		cb.filter(*q.index);

		return q;
	}

	ContainerPcRAW::QuaryMeta ContainerPcRAWOC::TestQuary(std::size_t i) const
	{
		if (i >= quaries.size())
			THROW_EXCEPTION("i is too large, max: " + std::to_string(quaries.size()));

		return quaries[i];
	}

	void ContainerPcRAWOC::LoadMeta()
	{
		std::string metaPath = (filePath / boost::filesystem::path("metaRAW.txt")).string();
		std::ifstream file(metaPath, std::ios_base::in);
		if (!file)
			THROW_EXCEPTION("Load file " + metaPath + " failed.");
		nlohmann::json j;
		file >> j;
		
		//
		if (j.find("overlap") == j.end())
			THROW_EXCEPTION("metaRAW is not valid: missing \"overlap\"");
		overlap = j["overlap"];

		for (nlohmann::json::const_iterator qit = j["quaries"].begin(); qit != j["quaries"].end(); ++qit)
		{
			QuaryMeta q;
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

		//
		file.close();
	}

	void ContainerPcRAWOC::DumpMeta() const
	{
		std::string metaPath = (filePath / boost::filesystem::path("metaRAW.txt")).string();
		std::ofstream file(metaPath, std::ios_base::out);
		if (!file)
			THROW_EXCEPTION("Create file " + metaPath + " failed.");
		nlohmann::json j;
		
		//
		j["overlap"] = overlap;

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

			j["quaries"].push_back(jQ);
		}

		//
		file << j;
		file.close();
	}
}
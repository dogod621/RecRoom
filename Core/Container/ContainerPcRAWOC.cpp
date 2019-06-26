#include <fstream>

#include <pcl/filters/crop_box.h>

#include "ContainerPcRAWOC.h"

namespace RecRoom
{
	ContainerPcRAWOC::ContainerPcRAWOC(const boost::filesystem::path& filePath_, 
		const Eigen::Vector3d& min, const Eigen::Vector3d& max, const double res, double overlap)
		: DumpAble("ContainerPcRAWOC", filePath_), ContainerPcRAW(), oct(nullptr), overlap(overlap), quaries()
	{
		if (CheckNew())
		{
			oct = OCT::Ptr(new OCT(min, max, res, filePath / boost::filesystem::path("pcRAW") / boost::filesystem::path("root.oct_idx"), "ECEF"));
			Dump();
		}
		else
		{
			oct = OCT::Ptr(new OCT(filePath / boost::filesystem::path("pcRAW") / boost::filesystem::path("root.oct_idx"), true));
			Load();
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
		Dump();
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

	void ContainerPcRAWOC::Load(const nlohmann::json& j)
	{
		if (j.find("overlap") == j.end())
			THROW_EXCEPTION("File is not valid: missing \"overlap\"");
		overlap = j["overlap"];

		for (nlohmann::json::const_iterator qit = j["quaries"].begin(); qit != j["quaries"].end(); ++qit)
		{
			QuaryMeta q;
			{
				if (qit->find("minAABB") == qit->end())
					THROW_EXCEPTION("File is not valid: missing \"minAABB\"");
				nlohmann::json::const_iterator it = (*qit)["minAABB"].begin();
				q.minAABB.x() = *it; ++it;
				q.minAABB.y() = *it; ++it;
				q.minAABB.z() = *it; ++it;
			}

			{
				if (qit->find("maxAABB") == qit->end())
					THROW_EXCEPTION("File is not valid: missing \"maxAABB\"");
				nlohmann::json::const_iterator it = (*qit)["maxAABB"].begin();
				q.maxAABB.x() = *it; ++it;
				q.maxAABB.y() = *it; ++it;
				q.maxAABB.z() = *it; ++it;
			}

			{
				if (qit->find("extMinAABB") == qit->end())
					THROW_EXCEPTION("File is not valid: missing \"extMinAABB\"");
				nlohmann::json::const_iterator it = (*qit)["extMinAABB"].begin();
				q.extMinAABB.x() = *it; ++it;
				q.extMinAABB.y() = *it; ++it;
				q.extMinAABB.z() = *it; ++it;
			}

			{
				if (qit->find("extMaxAABB") == qit->end())
					THROW_EXCEPTION("File is not valid: missing \"extMaxAABB\"");
				nlohmann::json::const_iterator it = (*qit)["extMaxAABB"].begin();
				q.extMaxAABB.x() = *it; ++it;
				q.extMaxAABB.y() = *it; ++it;
				q.extMaxAABB.z() = *it; ++it;
			}

			if (qit->find("depth") == qit->end())
				THROW_EXCEPTION("File is not valid: missing \"depth\"");
			q.depth = (*qit)["depth"];

			quaries.push_back(q);
		}
	}

	void ContainerPcRAWOC::Dump(nlohmann::json& j) const
	{
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
	}

	bool ContainerPcRAWOC::CheckNew() const
	{
		if(DumpAble::CheckNew())
			return true;
		else if (!boost::filesystem::is_directory(filePath / boost::filesystem::path("pcRAW")))
			return true;
		else if (!boost::filesystem::exists(filePath / boost::filesystem::path("pcRAW") / boost::filesystem::path("root.oct_idx")))
			return true;
		return false;
	}
}
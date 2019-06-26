#include <fstream>

#include <pcl/filters/crop_box.h>

#include "ContainerPcRAWOC.h"

namespace RecRoom
{
	ContainerPcRAWOC::ContainerPcRAWOC(const boost::filesystem::path& filePath_, 
		const Eigen::Vector3d& min, const Eigen::Vector3d& max, const double res, double overlap)
		: DumpAble("ContainerPcRAWOC", filePath_), ContainerPcRAW(), oct(nullptr), overlap(overlap), metaSet()
	{
		if (CheckExist())
		{
			oct = OCT::Ptr(new OCT(filePath / boost::filesystem::path("pcRAW") / boost::filesystem::path("root.oct_idx"), true));
			Load();
		}
		else
		{
			oct = OCT::Ptr(new OCT(min, max, res, filePath / boost::filesystem::path("pcRAW") / boost::filesystem::path("root.oct_idx"), "ECEF"));
			Dump();
		}

		//
		if (!oct)
			THROW_EXCEPTION("oct is not created?")
	}

	void ContainerPcRAWOC::Merge(const PTR(PcRAW)& v)
	{
		oct->addPointCloud(v);

		// update 
		metaSet.clear();
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
				metaSet.push_back(Meta(minAABB, maxAABB, minAABB - ext, maxAABB + ext, (*it)->getDepth()));
			}
			it++;
		}

		//
		Dump();
	}

	ContainerPcRAW::Meta ContainerPcRAWOC::GetMeta(std::size_t i) const
	{
		if (i >= metaSet.size())
			THROW_EXCEPTION("i is too large, max: " + std::to_string(metaSet.size()));

		return metaSet[i];
	}

	ContainerPcRAW::Data ContainerPcRAWOC::GetData(std::size_t i) const
	{
		if (i >= metaSet.size())
			THROW_EXCEPTION("i is too large, max: " + std::to_string(metaSet.size()));

		const Meta& meta =  metaSet[i];
		ContainerPcRAW::Data data (meta);

		//
		PTR(PcRAW) temp(new PcRAW);
		pcl::PCLPointCloud2::Ptr blob(new pcl::PCLPointCloud2);
		oct->queryBoundingBox(meta.extMinAABB, meta.extMaxAABB, meta.depth, blob);
		pcl::fromPCLPointCloud2(*blob, *temp);

		data.pcMED->resize(temp->size());
		for (std::size_t px = 0; px < temp->size(); ++px)
			(*data.pcMED)[px] = (*temp)[px];

		//
		pcl::CropBox<PointMED> cb;
		cb.setMin(Eigen::Vector4f(meta.minAABB.x(), meta.minAABB.y(), meta.minAABB.z(), 1.0));
		cb.setMax(Eigen::Vector4f(meta.maxAABB.x(), meta.maxAABB.y(), meta.maxAABB.z(), 1.0));
		cb.setInputCloud(data.pcMED);
		cb.filter(*data.pcIndex);

		return data;
	}

	void ContainerPcRAWOC::Load(const nlohmann::json& j)
	{
		if (j.find("overlap") == j.end())
			THROW_EXCEPTION("File is not valid: missing \"overlap\"");
		overlap = j["overlap"];

		for (nlohmann::json::const_iterator jMetaIt = j["metaSet"].begin(); jMetaIt != j["metaSet"].end(); ++jMetaIt)
		{
			Meta meta;
			{
				if (jMetaIt->find("minAABB") == jMetaIt->end())
					THROW_EXCEPTION("File is not valid: missing \"minAABB\"");
				nlohmann::json::const_iterator it = (*jMetaIt)["minAABB"].begin();
				meta.minAABB.x() = *it; ++it;
				meta.minAABB.y() = *it; ++it;
				meta.minAABB.z() = *it; ++it;
			}

			{
				if (jMetaIt->find("maxAABB") == jMetaIt->end())
					THROW_EXCEPTION("File is not valid: missing \"maxAABB\"");
				nlohmann::json::const_iterator it = (*jMetaIt)["maxAABB"].begin();
				meta.maxAABB.x() = *it; ++it;
				meta.maxAABB.y() = *it; ++it;
				meta.maxAABB.z() = *it; ++it;
			}

			{
				if (jMetaIt->find("extMinAABB") == jMetaIt->end())
					THROW_EXCEPTION("File is not valid: missing \"extMinAABB\"");
				nlohmann::json::const_iterator it = (*jMetaIt)["extMinAABB"].begin();
				meta.extMinAABB.x() = *it; ++it;
				meta.extMinAABB.y() = *it; ++it;
				meta.extMinAABB.z() = *it; ++it;
			}

			{
				if (jMetaIt->find("extMaxAABB") == jMetaIt->end())
					THROW_EXCEPTION("File is not valid: missing \"extMaxAABB\"");
				nlohmann::json::const_iterator it = (*jMetaIt)["extMaxAABB"].begin();
				meta.extMaxAABB.x() = *it; ++it;
				meta.extMaxAABB.y() = *it; ++it;
				meta.extMaxAABB.z() = *it; ++it;
			}

			if (jMetaIt->find("depth") == jMetaIt->end())
				THROW_EXCEPTION("File is not valid: missing \"depth\"");
			meta.depth = (*jMetaIt)["depth"];

			metaSet.push_back(meta);
		}
	}

	void ContainerPcRAWOC::Dump(nlohmann::json& j) const
	{
		j["overlap"] = overlap;

		for (std::size_t i = 0; i < metaSet.size(); ++i)
		{
			nlohmann::json jMeta;
			const Meta& meta = metaSet[i];

			jMeta["minAABB"].push_back(meta.minAABB.x());
			jMeta["minAABB"].push_back(meta.minAABB.y());
			jMeta["minAABB"].push_back(meta.minAABB.z());

			jMeta["maxAABB"].push_back(meta.maxAABB.x());
			jMeta["maxAABB"].push_back(meta.maxAABB.y());
			jMeta["maxAABB"].push_back(meta.maxAABB.z());

			jMeta["extMinAABB"].push_back(meta.extMinAABB.x());
			jMeta["extMinAABB"].push_back(meta.extMinAABB.y());
			jMeta["extMinAABB"].push_back(meta.extMinAABB.z());

			jMeta["extMaxAABB"].push_back(meta.extMaxAABB.x());
			jMeta["extMaxAABB"].push_back(meta.extMaxAABB.y());
			jMeta["extMaxAABB"].push_back(meta.extMaxAABB.z());

			jMeta["depth"] = meta.depth;

			j["metaSet"].push_back(jMeta);
		}
	}

	bool ContainerPcRAWOC::CheckExist() const
	{
		if(!DumpAble::CheckExist())
			return false;
		if (!boost::filesystem::is_directory(filePath / boost::filesystem::path("pcRAW")))
			return false;
		if (!boost::filesystem::exists(filePath / boost::filesystem::path("pcRAW") / boost::filesystem::path("root.oct_idx")))
			return false;
		return true;
	}
}
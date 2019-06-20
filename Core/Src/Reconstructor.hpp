#pragma once

#include <pcl/io/pcd_io.h>

#include "StringUtils.h"
#include "Reconstructor.h"
#include "AsyncProcess.h"

namespace RecRoom
{
	template<class PointRaw, class PointMed, class PointRec, class MetaScan>
	double Reconstructor<PointRaw, PointMed, PointRec, MetaScan>::SearchRadius() const
	{
		double r = 0;
		if (downSampler)r = std::max(r, downSampler->SearchRadius());
		if (outlierRemover)r = std::max(r, outlierRemover->SearchRadius());
		if (surfaceProcesser)r = std::max(r, surfaceProcesser->SearchRadius());
		if (normalEstimater)r = std::max(r, normalEstimater->SearchRadius());
		if (albedoEstimater)r = std::max(r, albedoEstimater->SearchRadius());
		if (segmenter)r = std::max(r, segmenter->SearchRadius());
		return r;
	}

	template<>
	inline ReconstructStatus Convert<ReconstructStatus, std::string>(const std::string& v)
	{
		if (v == "POINT_CLOUD") return ReconstructStatus::POINT_CLOUD;
		else if (v == "ALBEDO") return ReconstructStatus::ALBEDO;
		else if (v == "SEGMENT") return ReconstructStatus::SEGMENT;
		else if (v == "NDF") return ReconstructStatus::NDF;
		else if (v == "SURFACE") return ReconstructStatus::SURFACE;
		else return ReconstructStatus::ReconstructStatus_UNKNOWN;
	}

	template<>
	inline std::string Convert<std::string, ReconstructStatus>(const ReconstructStatus& v)
	{
		switch (v)
		{
		case ReconstructStatus::POINT_CLOUD: return std::string("POINT_CLOUD"); break;
		case ReconstructStatus::ALBEDO: return std::string("ALBEDO"); break;
		case ReconstructStatus::SEGMENT: return std::string("SEGMENT"); break;
		case ReconstructStatus::NDF: return std::string("NDF"); break;
		case ReconstructStatus::SURFACE: return std::string("SURFACE"); break;
		default: return std::string("UNKNOWN"); break;
		}
	}

	template<>
	inline ReconstructStatus Convert<ReconstructStatus, nlohmann::json>(const nlohmann::json& v)
	{
		ReconstructStatus r = ReconstructStatus::ReconstructStatus_UNKNOWN;
		for (nlohmann::json::const_iterator it = v.begin(); it != v.end(); it++)
			r = (ReconstructStatus)(r | Convert<ReconstructStatus, std::string>(*it));
		return r;
	}

	template<>
	inline nlohmann::json Convert<nlohmann::json, ReconstructStatus>(const ReconstructStatus& v)
	{
		nlohmann::json j;
		if (v & ReconstructStatus::POINT_CLOUD) j.push_back(Convert<std::string, ReconstructStatus>(ReconstructStatus::POINT_CLOUD));
		else if (v & ReconstructStatus::ALBEDO) j.push_back(Convert<std::string, ReconstructStatus>(ReconstructStatus::ALBEDO));
		else if (v & ReconstructStatus::SEGMENT) j.push_back(Convert<std::string, ReconstructStatus>(ReconstructStatus::SEGMENT));
		else if (v & ReconstructStatus::NDF) j.push_back(Convert<std::string, ReconstructStatus>(ReconstructStatus::NDF));
		else if (v & ReconstructStatus::SURFACE) j.push_back(Convert<std::string, ReconstructStatus>(ReconstructStatus::SURFACE));
		return j;
	}

	template<class PointRaw, class PointMed, class PointRec, class MetaScan>
	Reconstructor<PointRaw, PointMed, PointRec, MetaScan>::Reconstructor(const boost::filesystem::path& filePath, const Eigen::Vector3d& min, const Eigen::Vector3d& max, const double resolution)
		: Data(), filePath(filePath), metaScans(new MetaScansT), status(ReconstructStatus::ReconstructStatus_UNKNOWN),
		containerRaw(nullptr), containerNDF(nullptr), 
		downSampler(nullptr), outlierRemover(nullptr), surfaceProcesser(nullptr), normalEstimater(nullptr), albedoEstimater(nullptr), segmenter(nullptr),
		pointCloudRec(pointCloudRec)
	{
		if (!boost::filesystem::exists(filePath))
		{
			boost::filesystem::create_directory(filePath);
			PRINT_INFO("Create directory: " + filePath.string());
		}

		containerRaw = ContainerRawT::Ptr(new ContainerRawT(min, max, resolution,
			filePath / boost::filesystem::path("RAW") / boost::filesystem::path("root.oct_idx"), "ECEF"));
		containerNDF = ContainerNDFT::Ptr(new ContainerNDFT(16, Eigen::Vector3d(-1.0, -1.0, -1.0), Eigen::Vector3d((double)std::numeric_limits<unsigned short>::max(), 1.0, 1.0),
			filePath / boost::filesystem::path("NDF") / boost::filesystem::path("root.oct_idx"), "ECEF"));
		pointCloudRec = PointCloudRecT::Ptr(new PointCloudRecT);
		pcl::io::savePCDFile((filePath / boost::filesystem::path("pointCloudRec.pcd")).string(), *pointCloudRec, true);

		std::string metaPath = (filePath / boost::filesystem::path("meta.txt")).string();
		std::ofstream file(metaPath, std::ios_base::out);
		if (!file)
			throw pcl::PCLException("Create file " + metaPath + " failed.");
		nlohmann::json metaJson;
		ToJson(metaJson);
		file << metaJson;
		file.close();
	}

	template<class PointRaw, class PointMed, class PointRec, class MetaScan>
	Reconstructor<PointRaw, PointMed, PointRec, MetaScan>::Reconstructor(const boost::filesystem::path& filePath)
		: Data(), filePath(filePath), metaScans(new MetaScansT), status(ReconstructStatus::ReconstructStatus_UNKNOWN),
		containerRaw(nullptr), containerNDF(nullptr),
		downSampler(nullptr), outlierRemover(nullptr), surfaceProcesser(nullptr), normalEstimater(nullptr), albedoEstimater(nullptr), segmenter(nullptr),
		pointCloudRec(pointCloudRec)
	{
		if (!IsFileRec(filePath, true))
			THROW_EXCEPTION("filePath is not valid.");

		containerRaw = ContainerRawT::Ptr(new ContainerRawT(filePath / boost::filesystem::path("RAW") / boost::filesystem::path("root.oct_idx"), true));
		containerNDF = ContainerNDFT::Ptr(new ContainerNDFT(filePath / boost::filesystem::path("NDF") / boost::filesystem::path("root.oct_idx"), true));
		pointCloudRec = PointCloudRecT::Ptr(new PointCloudRecT);
		pcl::io::loadPCDFile((filePath / boost::filesystem::path("pointCloudRec.pcd")).string(), *pointCloudRec);

		std::string metaPath = (filePath / boost::filesystem::path("meta.txt")).string();
		std::ifstream file(metaPath, std::ios_base::in);
		if (!file)
			throw pcl::PCLException("Load file " + metaPath + " failed.");
		nlohmann::json metaJson;
		FromJson(metaJson);
		file >> metaJson;
		file.close();
	}

	template<class PointRaw, class PointMed, class PointRec, class MetaScan>
	void Reconstructor<PointRaw, PointMed, PointRec, MetaScan>::ReconstructLOD(const double samplePercent)
	{
		containerRaw->setSamplePercent(samplePercent);
		containerRaw->buildLOD();
	}

	template<class PointRaw, class PointMed, class PointRec, class MetaScan>
	void Reconstructor<PointRaw, PointMed, PointRec, MetaScan>::FromJson(const nlohmann::json& j)
	{
		status = Convert<ReconstructStatus, nlohmann::json>(j["status"]);

		{
			MetaScansT metaScans_;
			for (nlohmann::json::const_iterator it = j["metaScans"].begin(); it != j["metaScans"].end(); ++it)
			{
				MetaScan::Ptr temp (new MetaScan);
				temp->FromJson(*it);
				temp->ClearBuffers();
				metaScans_.push_back(temp);
			}

			std::sort(metaScans_.begin(), metaScans_.end(), MetaScan::Compare);
			metaScans->resize(metaScans_.back()->getSerialNumber() + 1);
			std::size_t i = 0;
			for (std::vector<MetaScan::Ptr>::iterator it = metaScans_.begin(); it != metaScans_.end(); ++it)
			{
				if ((*it)->getSerialNumber() < 0)
				{
					PRINT_WARNING("MetaScan missging serial number, ignore it");
				}
				else
				{
					if ((*it)->getSerialNumber() < i)
					{
						PRINT_WARNING("Duplicate metaScan [" + std::to_string((*it)->getSerialNumber())  + "], ignore it");
					}
					else
					{
						if ((*it)->getSerialNumber() > i)
						{
							PRINT_WARNING("Missing metaScans [" + std::to_string(i) + ", " + std::to_string((*it)->getSerialNumber()) + ")");
						}
						(*metaScans)[(*it)->getSerialNumber()] = *it;
					}
					i = (*it)->getSerialNumber() + 1;
				}
			}
		}

		if (j.find("downSampler") != j.end()) downSampler->FromJson(j["downSampler"]);
		if (j.find("outlierRemover") != j.end()) outlierRemover->FromJson(j["outlierRemover"]);
		if (j.find("surfaceProcesser") != j.end()) surfaceProcesser->FromJson(j["surfaceProcesser"]);
		if (j.find("normalEstimater") != j.end()) normalEstimater->FromJson(j["normalEstimater"]);
		if (j.find("albedoEstimater") != j.end()) albedoEstimater->FromJson(j["albedoEstimater"]);
		if (j.find("segmenter") != j.end()) segmenter->FromJson(j["segmenter"]);
	}

	template<class PointRaw, class PointMed, class PointRec, class MetaScan>
	void Reconstructor<PointRaw, PointMed, PointRec, MetaScan>::ToJson(nlohmann::json& j) const
	{
		j["status"] = Convert<nlohmann::json, ReconstructStatus>(status);
		for (std::size_t i = 0; i < metaScans->size(); ++i)
		{
			if ((*metaScans)[i])
			{
				(*metaScans)[i]->ClearBuffers();
				(*metaScans)[i]->ToJson(j["metaScans"]);
			}
		}

		if(downSampler) downSampler->ToJson(j["downSampler"]);
		if (outlierRemover) outlierRemover->ToJson(j["outlierRemover"]);
		if (surfaceProcesser) surfaceProcesser->ToJson(j["surfaceProcesser"]);
		if (normalEstimater) normalEstimater->ToJson(j["normalEstimater"]);
		if (albedoEstimater) albedoEstimater->ToJson(j["albedoEstimater"]);
		if (segmenter) segmenter->ToJson(j["segmenter"]);
	}
}
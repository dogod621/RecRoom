#pragma once

#include <fstream>

#include "StringUtils.h"

#include "E57Reconstructor.h"
#include "AsyncProcess.h"

namespace RecRoom
{
	E57Reconstructor::E57Reconstructor(const boost::filesystem::path& filePath, const Eigen::Vector3d& min, const Eigen::Vector3d& max, const double resolution)
		: E57ReconstructorBase(), scanMeta(), filePath(filePath)
	{
		if (!boost::filesystem::exists(filePath))
		{
			boost::filesystem::create_directory(filePath);
			PRINT_INFO("Create directory: " + filePath.string());
		}

		{
			std::string scanMetaPath = (filePath / boost::filesystem::path("scanMeta.txt")).string();
			std::ofstream file(scanMetaPath, std::ios_base::out);
			if (!file)
				throw pcl::PCLException("Create file " + scanMetaPath + " failed.");
			nlohmann::json scanMetaJson;
			ToJson(scanMetaJson);
			file << scanMetaJson;
			file.close();
		}

		rawContainer = RAWContainer::Ptr(new RAWContainer(min, max, resolution,
			filePath / boost::filesystem::path("RAW") / boost::filesystem::path("root.oct_idx"), "ECEF"));
		ndfContainer = NDFContainer::Ptr(new NDFContainer(Eigen::Vector3d(-1.0, -1.0, -1.0), Eigen::Vector3d((double)std::numeric_limits<unsigned int>::max(), 1.0, 1.0), 1.0,
			filePath / boost::filesystem::path("NDF") / boost::filesystem::path("root.oct_idx"), "ECEF"));

	}

	
	E57Reconstructor::E57Reconstructor(const boost::filesystem::path& filePath)
		: filePath(filePath)
	{
		if (!IsFileE57Rec(filePath, true))
			THROW_EXCEPTION("filePath is not valid.");

		{
			std::string scanMetaPath = (filePath / boost::filesystem::path("scanMeta.txt")).string();
			std::ifstream file(scanMetaPath, std::ios_base::in);
			if (!file)
				throw pcl::PCLException("Load file " + scanMetaPath + " failed.");
			nlohmann::json scanMetaJson;
			FromJson(scanMetaJson);
			file >> scanMetaJson;
			file.close();
		}

		rawContainer = RAWContainer::Ptr(new RAWContainer(filePath / boost::filesystem::path("RAW") / boost::filesystem::path("root.oct_idx"), true));
		ndfContainer = NDFContainer::Ptr(new NDFContainer(filePath / boost::filesystem::path("NDF") / boost::filesystem::path("root.oct_idx"), true));
	}

	bool E57ScanDataCompare(const E57ScanData::Ptr& i, const E57ScanData::Ptr& j) { return (i->getSerialNumber() < j->getSerialNumber()); }
	void E57Reconstructor::FromJson(const nlohmann::json& j)
	{
		std::vector<E57ScanData::Ptr> rawScanMeta;
		rawScanMeta.resize(j.size());
		for (std::size_t i = 0; i < rawScanMeta.size(); ++i)
		{
			rawScanMeta[i] = E57ScanData::Ptr(new E57ScanData());
			rawScanMeta[i]->FromJson(j[i]);
		}

		std::sort(rawScanMeta.begin(), rawScanMeta.end(), E57ScanDataCompare);
		scanMeta.resize(rawScanMeta.back()->getSerialNumber() + 1);

		std::size_t i = 0;
		for (std::vector<E57ScanData::Ptr>::iterator it = rawScanMeta.begin(); it != rawScanMeta.end(); ++it)
		{
			if ((*it)->getSerialNumber() < 0)
			{
				PRINT_WARNING("ScanMeta missging serial number, ignore it");
			}
			else
			{
				scanMeta[(*it)->getSerialNumber()] = *it;
				if ((*it)->getSerialNumber() != i)
				{
					PRINT_WARNING("Missing scanMeta [" + std::to_string(i) + ", " + std::to_string((*it)->getSerialNumber()) + ")");
				}
				i = (*it)->getSerialNumber() + 1;
			}
		}
	}

	void E57Reconstructor::ToJson(nlohmann::json& j) const
	{
		for (std::size_t i = 0; i < scanMeta.size(); ++i)
		{
			if (scanMeta[i])
			{
				nlohmann::json metaJson;
				scanMeta[i]->ToJson(metaJson);
				j.push_back(metaJson);
			}
		}
	}

	struct Global_FromFile
	{
		e57::ImageFile* imf;
		e57::VectorNode* data3D;
		E57Reconstructor::RAWContainer::Ptr rawContainer;

		Global_FromFile(e57::ImageFile* imf = nullptr, e57::VectorNode* data3D = nullptr, E57Reconstructor::RAWContainer::Ptr rawContainer = nullptr)
			: imf(imf), data3D(data3D), rawContainer(rawContainer){}
	};

	struct Query_FromFile
	{
		int scanSerialNumber;
		Query_FromFile(int scanSerialNumber = -1)
			: scanSerialNumber(scanSerialNumber) {}
	};

	int A_FromFile(Global_FromFile& global, Query_FromFile& query, E57ScanData::Ptr& data)
	{
		PRINT_INFO("Start - " + std::to_string(query.scanSerialNumber));

		if (query.scanSerialNumber >= global.data3D->childCount()) return 1;

		data->FromE57Format(*global.imf, *global.data3D, query.scanSerialNumber);

		PRINT_INFO("End - " + std::to_string(query.scanSerialNumber));
		return 0;
	}

	int B_FromFile(Global_FromFile& global, Query_FromFile& query, E57ScanData::Ptr& data)
	{
		PRINT_INFO("Start - " + std::to_string(query.scanSerialNumber));

		if (!data) return 1;

		PRINT_INFO("End - " + std::to_string(query.scanSerialNumber));
		return 0;
	}

	int C_FromFile(Global_FromFile& global, E57ScanData::Ptr& data)
	{
		PRINT_INFO("Start");
		
		if (!data) return 1;
		
		E57Reconstructor::RawPointCloud::Ptr scanCloud = E57Reconstructor::RawPointCloud::Ptr(new E57Reconstructor::RawPointCloud);
		data->ToPointCloud(*scanCloud);
		global.rawContainer->addPointCloud(scanCloud);
		data->ClearBuffers();

		PRINT_INFO("End");
		return 0;
	}

	void E57Reconstructor::FromFile(const boost::filesystem::path& filePath)
	{
		if (!IsFileE57(filePath, true))
			THROW_EXCEPTION("filePath is not valid.");

		scanMeta.clear();
		e57::ImageFile imf(filePath.string().c_str(), "r");

		//
		if (imf.root().isDefined("data3D"))
		{
			e57::Node data3DNode = imf.root().get("data3D");

			if (data3DNode.type() == e57::NodeType::E57_VECTOR)
			{
				e57::VectorNode data3D(data3DNode);

				Global_FromFile global(&imf, &data3D, rawContainer);
				std::vector<Query_FromFile> queries(data3D.childCount());
				for (std::size_t i = 0; i < queries.size(); ++i)
					queries[i].scanSerialNumber = i;

				AsyncProcess<Global_FromFile, Query_FromFile, E57ScanData::Ptr, 1>(global, queries, A_FromFile, B_FromFile, C_FromFile);
			}
			else
				PRINT_WARNING("E57 file data3D is not vector, ignore it");
		}
		else
			PRINT_WARNING("E57 file did not define data3D, ignore it");

		//
		if (imf.root().isDefined("images2D"))
		{
			e57::Node images2DNode = imf.root().get("images2D");

			if (images2DNode.type() == e57::NodeType::E57_VECTOR)
			{
				e57::VectorNode images2D(images2DNode);

			}
			else
				PRINT_WARNING("E57 file images2D is not vector, ignore it");
		}
		else
			PRINT_WARNING("E57 file did not define images2D, ignore it");

		//
		imf.close();
	}

	void E57Reconstructor::ReconstructLOD(const double samplePercent) 
	{ 
		rawContainer->setSamplePercent(samplePercent);
		rawContainer->buildLOD();
	}
}
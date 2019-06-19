#pragma once

#include <fstream>
#include <algorithm> // std::set_intersection, std::sort

#include <pcl/filters/crop_box.h>

#include "StringUtils.h"

#include "E57Reconstructor.h"
#include "AsyncProcess.h"

namespace RecRoom
{
	E57Reconstructor::E57Reconstructor(const boost::filesystem::path& filePath, const Eigen::Vector3d& min, const Eigen::Vector3d& max, const double resolution, const double outofCoreLeafOverlap_)
		: Base(filePath, min, max, resolution, outofCoreLeafOverlap_), scanMeta()
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

	E57Reconstructor::E57Reconstructor(const boost::filesystem::path& filePath, const double outofCoreLeafOverlap_)
		: Base(filePath, outofCoreLeafOverlap_), scanMeta()
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
		boost::shared_ptr<e57::ImageFile> imf;
		boost::shared_ptr<e57::VectorNode> data3D;
		E57Reconstructor::RAWContainer::Ptr rawContainer;

		Global_FromFile(boost::shared_ptr<e57::ImageFile> imf = nullptr, boost::shared_ptr<e57::VectorNode> data3D = nullptr, E57Reconstructor::RAWContainer::Ptr rawContainer = nullptr)
			: imf(imf), data3D(data3D), rawContainer(rawContainer) {}
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
		data = E57ScanData::Ptr(new E57ScanData);
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
		boost::shared_ptr<e57::ImageFile> imf ( new e57::ImageFile(filePath.string().c_str(), "r"));

		//
		if (imf->root().isDefined("data3D"))
		{
			e57::Node data3DNode = imf->root().get("data3D");

			if (data3DNode.type() == e57::NodeType::E57_VECTOR)
			{
				boost::shared_ptr<e57::VectorNode> data3D( new e57::VectorNode(data3DNode));

				Global_FromFile global(imf, data3D, rawContainer);
				std::vector<Query_FromFile> queries(data3D->childCount());
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
		if (imf->root().isDefined("images2D"))
		{
			e57::Node images2DNode = imf->root().get("images2D");

			if (images2DNode.type() == e57::NodeType::E57_VECTOR)
			{
				boost::shared_ptr<e57::VectorNode> images2D(new e57::VectorNode(images2DNode));

			}
			else
				PRINT_WARNING("E57 file images2D is not vector, ignore it");
		}
		else
			PRINT_WARNING("E57 file did not define images2D, ignore it");

		//
		imf->close();
	}
	
	struct Global_Reconstruct
	{
		E57Reconstructor::RAWContainer::Ptr rawContainer;
		E57Reconstructor::NDFContainer::Ptr ndfContainer;
		E57Reconstructor::DownSampler::Ptr downSampler;
		E57Reconstructor::OutlierRemover::Ptr outlierRemover;
		E57Reconstructor::SurfaceProcesser::Ptr surfaceProcesser;
		E57Reconstructor::NormalEstimater::Ptr normalEstimater;
		E57Reconstructor::AlbedoEstimater::Ptr albedoEstimater;
		E57Reconstructor::Segmenter::Ptr segmenter;
		E57Reconstructor::RecPointCloud::Ptr recPointCloud;

		Global_Reconstruct(
			E57Reconstructor::RAWContainer::Ptr rawContainer = nullptr,
			E57Reconstructor::NDFContainer::Ptr ndfContainer = nullptr,
			E57Reconstructor::DownSampler::Ptr downSampler = nullptr,
			E57Reconstructor::OutlierRemover::Ptr outlierRemover = nullptr,
			E57Reconstructor::SurfaceProcesser::Ptr surfaceProcesser = nullptr,
			E57Reconstructor::NormalEstimater::Ptr normalEstimater = nullptr,
			E57Reconstructor::AlbedoEstimater::Ptr albedoEstimater = nullptr,
			E57Reconstructor::Segmenter::Ptr segmenter = nullptr,
			E57Reconstructor::RecPointCloud::Ptr recPointCloud = nullptr)
			: rawContainer(rawContainer), ndfContainer(ndfContainer),
			downSampler(downSampler), outlierRemover(outlierRemover),
			surfaceProcesser(surfaceProcesser), normalEstimater(normalEstimater), albedoEstimater(albedoEstimater), segmenter(segmenter),
			recPointCloud(recPointCloud)
		{}
	};

	struct Query_Reconstruct
	{
		Eigen::Vector3d minBB;
		Eigen::Vector3d maxBB;
		Eigen::Vector3d extMinBB;
		Eigen::Vector3d extMaxBB;
		std::size_t depth;

		Query_Reconstruct(
			const Eigen::Vector3d& minBB = Eigen::Vector3d(0.0, 0.0, 0.0),
			const Eigen::Vector3d& maxBB = Eigen::Vector3d(0.0, 0.0, 0.0),
			std::size_t depth = 0)
			: minBB(minBB), maxBB(maxBB), depth(depth)
		{}

		void InitExtBB(double outofCoreLeafOverlap)
		{
			Eigen::Vector3d extXYZ(outofCoreLeafOverlap, outofCoreLeafOverlap, outofCoreLeafOverlap);
			extMinBB = minBB - extXYZ;
			extMaxBB = maxBB + extXYZ;
		}
	};

	struct Data_Reconstruct
	{
		pcl::search::KdTree<E57Reconstructor::RawPointType>::Ptr rawPointCloudTree;
		pcl::search::KdTree<E57Reconstructor::MedPointType>::Ptr medPointCloudTree;
		pcl::search::KdTree<E57Reconstructor::RecPointType>::Ptr recPointCloudTree;
		E57Reconstructor::RawPointCloud::Ptr rawPointCloud;
		E57Reconstructor::MedPointCloud::Ptr medPointCloud;
		E57Reconstructor::RecPointCloud::Ptr recPointCloud;
		pcl::IndicesPtr medPointCloudIndices;
	};

	int A_ReconstructPointCloud(Global_Reconstruct& global, Query_Reconstruct& query, Data_Reconstruct& data)
	{
		std::stringstream strQuery;
		strQuery << query.extMinBB << ", " << query.extMaxBB;
		PRINT_INFO("Start - " + strQuery.str());

		// Init
		data.rawPointCloud = E57Reconstructor::RawPointCloud::Ptr(new E57Reconstructor::RawPointCloud);
		data.rawPointCloudTree = pcl::search::KdTree<E57Reconstructor::RawPointType>::Ptr(new pcl::search::KdTree<E57Reconstructor::RawPointType>());
		data.rawPointCloudTree->setInputCloud(data.rawPointCloud);

		data.medPointCloud = E57Reconstructor::MedPointCloud::Ptr(new E57Reconstructor::MedPointCloud);
		data.medPointCloudTree = pcl::search::KdTree<E57Reconstructor::MedPointType>::Ptr(new pcl::search::KdTree<E57Reconstructor::MedPointType>());
		data.medPointCloudTree->setInputCloud(data.medPointCloud);
		data.recPointCloud = E57Reconstructor::RecPointCloud::Ptr(new  E57Reconstructor::RecPointCloud);
		data.recPointCloudTree = pcl::search::KdTree<E57Reconstructor::RecPointType>::Ptr(new pcl::search::KdTree<E57Reconstructor::RecPointType>());
		data.recPointCloudTree->setInputCloud(data.recPointCloud);

		data.medPointCloudIndices = pcl::IndicesPtr(new std::vector<int>);

		// Raw
		PRINT_INFO("Load rawPointCloud");
		pcl::PCLPointCloud2::Ptr blob(new pcl::PCLPointCloud2);
		global.rawContainer->queryBoundingBox(query.extMinBB, query.extMaxBB, query.depth, blob);
		pcl::fromPCLPointCloud2(*blob, *data.rawPointCloud);

		// Med
		if (global.downSampler)
		{
			PRINT_INFO("DownSampling medPointCloud");
			E57Reconstructor::RawPointCloud::Ptr tempMedPointCloud(new E57Reconstructor::RawPointCloud);
			global.downSampler->setInputCloud(data.rawPointCloud);
			global.downSampler->filter(*tempMedPointCloud);

			data.medPointCloud->resize(tempMedPointCloud->size());
			for (std::size_t px = 0; px < tempMedPointCloud->size(); ++px)
				(*data.medPointCloud)[px] = (*tempMedPointCloud)[px];

			std::stringstream ss;
			ss << "DownSampling medPointCloud - inSize, outSize: " << data.rawPointCloud->size() << ", " << data.medPointCloud->size();
			PRINT_INFO(ss.str());
		}
		else
		{
			data.medPointCloud->resize(data.rawPointCloud->size());
			for (std::size_t px = 0; px < data.rawPointCloud->size(); ++px)
				(*data.medPointCloud)[px] = (*data.rawPointCloud)[px];
		}

		// Med Indices
		{
			PRINT_INFO("Extract medPointCloud indices");

			// Outlier Removal
			std::vector<int> orIndices;
			if (global.outlierRemover)
			{
				PRINT_INFO("Extract medPointCloud indices - Outlier Removal");
				global.outlierRemover->setInputCloud(data.medPointCloud);
				global.outlierRemover->filter(orIndices);
				std::sort(orIndices.begin(), orIndices.end());
				std::stringstream ss;
				ss << "Extract medPointCloud indices - Outlier Removal - inSize, outSize: " << data.medPointCloud->size() << ", " << orIndices.size();
				PRINT_INFO(ss.str());
			}

			// Crop
			std::vector<int> cropIndices;
			{
				PRINT_INFO("Extract medPointCloud indices - Crop");

				pcl::CropBox<E57Reconstructor::MedPointType> cb;
				cb.setMin(Eigen::Vector4f(query.minBB.x(), query.minBB.y(), query.minBB.z(), 1.0));
				cb.setMax(Eigen::Vector4f(query.maxBB.x(), query.maxBB.y(), query.maxBB.z(), 1.0));
				cb.setInputCloud(data.medPointCloud);
				cb.filter(cropIndices);
				std::sort(cropIndices.begin(), cropIndices.end());
				std::stringstream ss;
				ss << "Extract medPointCloud indices - Crop - inSize, outSize: " << data.medPointCloud->size() << ", " << cropIndices.size();
				PRINT_INFO(ss.str());
			}

			// Combine
			{
				data.medPointCloudIndices->resize(std::min(orIndices.size(), cropIndices.size()));
				std::vector<int>::iterator it = std::set_intersection(
					orIndices.begin(), orIndices.end(),
					cropIndices.begin(), cropIndices.end(),
					data.medPointCloudIndices->begin());
				data.medPointCloudIndices->resize(it - data.medPointCloudIndices->begin());

				std::stringstream ss;
				ss << "Extract medPointCloud indices - inSize, outSize: " << data.medPointCloud->size() << ", " << data.medPointCloudIndices->size();
				PRINT_INFO(ss.str());
			}
		}

		//
		PRINT_INFO("End - " + strQuery.str());
		return 0;
	}

	int B_ReconstructPointCloud(Global_Reconstruct& global, Query_Reconstruct& query, Data_Reconstruct& data)
	{
		std::stringstream strQuery;
		strQuery << query.minBB << ", " << query.maxBB;
		PRINT_INFO("Start - " + strQuery.str());

		if (!data.rawPointCloud) return 1;
		if (!data.medPointCloud) return 2;
		if (!data.recPointCloud) return 3;
		if (!data.rawPointCloudTree) return 4;
		if (!data.medPointCloudTree) return 5;
		if (!data.recPointCloudTree) return 6;
		if (!data.medPointCloudIndices) return 7;

		// Estimat Normal
		if (global.normalEstimater)
		{
			if (E57xPCD_CAN_CONTAIN_NORMAL)
			{
				PRINT_INFO("Estimat Normal medPointCloud");

				global.normalEstimater->setSearchMethod(data.medPointCloudTree);
				global.normalEstimater->setSearchSurface(data.medPointCloud);
				global.normalEstimater->setInputCloud(data.medPointCloud);
				global.normalEstimater->setIndices(data.medPointCloudIndices);
				global.normalEstimater->compute(*data.medPointCloud);
			}
		}

		// Extract
		{
			E57Reconstructor::MedPointCloud::Ptr tempMedPointCloud(new E57Reconstructor::MedPointCloud);
			pcl::ExtractIndices<E57Reconstructor::MedPointType> extract;
			extract.setInputCloud(data.medPointCloud);
			extract.setIndices(data.medPointCloudIndices);
			extract.filter(*tempMedPointCloud);

			//
			data.recPointCloud->resize(tempMedPointCloud->size());
			for (std::size_t px = 0; px < tempMedPointCloud->size(); ++px)
				(*data.recPointCloud)[px] = (*tempMedPointCloud)[px];
		}

		PRINT_INFO("End - " + strQuery.str());
		return 0;
	}

	int C_ReconstructPointCloud(Global_Reconstruct& global, Data_Reconstruct& data)
	{
		PRINT_INFO("Start");

		if (!data.rawPointCloud) return 1;
		if (!data.medPointCloud) return 2;
		if (!data.recPointCloud) return 3;
		if (!data.rawPointCloudTree) return 4;
		if (!data.medPointCloudTree) return 5;
		if (!data.recPointCloudTree) return 6;
		if (!data.medPointCloudIndices) return 7;

		(*global.recPointCloud) += (*data.recPointCloud);

		PRINT_INFO("End");
		return 0;
	}

	void E57Reconstructor::ReconstructPointCloud()
	{
		if (!recPointCloud)
			THROW_EXCEPTION("pointCloud is not set.");

		//
		recPointCloud->clear();
		Global_Reconstruct global(
			rawContainer, ndfContainer,
			downSampler, outlierRemover,
			surfaceProcesser, normalEstimater, albedoEstimater, segmenter,
			recPointCloud);

		//
		std::vector<Query_Reconstruct> queries;
		RAWContainer::Iterator it(*rawContainer);
		while (*it != nullptr)
		{
			if ((*it)->getNodeType() == pcl::octree::LEAF_NODE)
			{
				Query_Reconstruct query;
				(*it)->getBoundingBox(query.minBB, query.maxBB);
				query.depth = (*it)->getDepth();
				query.InitExtBB(outofCoreLeafOverlap);
				queries.push_back(query);
			}
			it++;
		}
		AsyncProcess<Global_Reconstruct, Query_Reconstruct, Data_Reconstruct, 1>(global, queries, A_ReconstructPointCloud, B_ReconstructPointCloud, C_ReconstructPointCloud);
	}
}
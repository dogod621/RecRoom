#pragma once

#include <fstream>
#include <algorithm> // std::set_intersection, std::sort

#include <pcl/filters/crop_box.h>

#include "StringUtils.h"

#include "ReconstructorE57.h"
#include "AsyncProcess.h"

namespace RecRoom
{
	ReconstructorE57::ReconstructorE57(const boost::filesystem::path& filePath, const Eigen::Vector3d& min, const Eigen::Vector3d& max, const double resolution)
		: Reconstructor<PointE57, PointE57xPCD, PointPCD, DataScanE57>(filePath, min, max, resolution)
	{}

	ReconstructorE57::ReconstructorE57(const boost::filesystem::path& filePath)
		: Reconstructor<PointE57, PointE57xPCD, PointPCD, DataScanE57>(filePath)
	{
		if (!IsFileE57Rec(filePath, true))
			THROW_EXCEPTION("filePath is not valid.");
	}

	struct Global_FromFile
	{
		e57::ImageFile* imf;
		e57::VectorNode* data3D;
		ReconstructorE57* reconstructorE57;

		Global_FromFile(e57::ImageFile* imf = nullptr, e57::VectorNode* data3D = nullptr, ReconstructorE57* reconstructorE57 = nullptr)
			: imf(imf), data3D(data3D), reconstructorE57(reconstructorE57) {}

		int Check()
		{
			if (imf == nullptr) return 1;
			if (data3D == nullptr) return 2;
			if (reconstructorE57 == nullptr) return 3;
			return 0;
		}
	};

	struct Query_FromFile
	{
		int scanSerialNumber;
		Query_FromFile(int scanSerialNumber = -1)
			: scanSerialNumber(scanSerialNumber) {}
	};

	using Data_FromFile = DataScanE57;

	int A_FromFile(Global_FromFile& global, Query_FromFile& query, Data_FromFile& data)
	{
		int cerr = global.Check();
		if (cerr == 0)
		{
			PRINT_INFO("Start - " + std::to_string(query.scanSerialNumber));

			if (query.scanSerialNumber >= global.data3D->childCount()) return 1;
			data.FromE57Format(*global.imf, *global.data3D, query.scanSerialNumber);

			PRINT_INFO("End - " + std::to_string(query.scanSerialNumber));
		}
		return cerr;
	}

	int B_FromFile(Global_FromFile& global, Query_FromFile& query, Data_FromFile& data)
	{
		int cerr = global.Check();
		if (cerr == 0)
		{
			PRINT_INFO("Start - " + std::to_string(query.scanSerialNumber));
			PRINT_INFO("End - " + std::to_string(query.scanSerialNumber));
		}
		return cerr;
	}

	int C_FromFile(Global_FromFile& global, Data_FromFile& data)
	{
		int cerr = global.Check();
		if (cerr == 0)
		{
			PRINT_INFO("Start");

			ReconstructorE57::PointCloudRawT::Ptr scanCloud = ReconstructorE57::PointCloudRawT::Ptr(new ReconstructorE57::PointCloudRawT);
			data.ToPointCloud(*scanCloud);
			global.reconstructorE57->getContainerRaw()->addPointCloud(scanCloud);
			data.ClearBuffers();
			*(*global.reconstructorE57->getMetaScans())[data.getSerialNumber()] = data;

			PRINT_INFO("End");
		}
		return cerr;
	}

	void ReconstructorE57::FromFile(const boost::filesystem::path& filePath)
	{
		if (!IsFileE57(filePath, true))
			THROW_EXCEPTION("filePath is not valid.");

		metaScans->clear();
		e57::ImageFile imf(filePath.string().c_str(), "r");

		//
		if (imf.root().isDefined("data3D"))
		{
			e57::Node data3DNode = imf.root().get("data3D");

			if (data3DNode.type() == e57::NodeType::E57_VECTOR)
			{
				e57::VectorNode data3D(data3DNode);

				Global_FromFile global(&imf, &data3D, this);
				std::vector<Query_FromFile> queries(data3D.childCount());
				for (std::size_t i = 0; i < queries.size(); ++i)
					queries[i].scanSerialNumber = i;

				AsyncProcess<Global_FromFile, Query_FromFile, Data_FromFile, 1>(global, queries, A_FromFile, B_FromFile, C_FromFile);
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


	struct Global_Reconstruct
	{
		ReconstructorE57* reconstructorE57;

		Global_Reconstruct(ReconstructorE57* reconstructorE57 = nullptr)
			: reconstructorE57(reconstructorE57) {}

		int Check()
		{
			if (reconstructorE57 == nullptr) return 1;
			return 0;
		}
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
			std::size_t depth = 0, double outofCoreLeafOverlap = -1)
			: minBB(minBB), maxBB(maxBB), depth(depth)
		{
			InitExtBB(outofCoreLeafOverlap);
		}

		void InitExtBB(double outofCoreLeafOverlap)
		{
			if (outofCoreLeafOverlap > 0)
			{
				Eigen::Vector3d extXYZ(outofCoreLeafOverlap, outofCoreLeafOverlap, outofCoreLeafOverlap);
				extMinBB = minBB - extXYZ;
				extMaxBB = maxBB + extXYZ;
			}
		}
	};

	struct Data_Reconstruct
	{
		ReconstructorE57::DownSamplerT::Ptr downSampler;
		ReconstructorE57::OutlierRemoverT::Ptr outlierRemover;
		ReconstructorE57::SurfaceProcesserT::Ptr surfaceProcesser;
		ReconstructorE57::NormalEstimaterT::Ptr normalEstimater;
		ReconstructorE57::AlbedoEstimaterT::Ptr albedoEstimater;
		ReconstructorE57::SegmenterT::Ptr segmenter;

		ReconstructorE57::PointCloudRawT::Ptr pointCloudRaw;
		ReconstructorE57::PointCloudMedT::Ptr pointCloudMed;
		ReconstructorE57::PointCloudRecT::Ptr pointCloudRec;
		pcl::IndicesPtr indicesMed;
		pcl::search::KdTree<ReconstructorE57::PointRawT>::Ptr treeRaw;
		pcl::search::KdTree<ReconstructorE57::PointMedT>::Ptr treeMed;
		pcl::search::KdTree<ReconstructorE57::PointRecT>::Ptr treeRec;

		Data_Reconstruct() :
			downSampler(nullptr), outlierRemover(nullptr), surfaceProcesser(nullptr), normalEstimater(nullptr), albedoEstimater(nullptr), segmenter(nullptr),
			pointCloudRaw(ReconstructorE57::PointCloudRawT::Ptr(new ReconstructorE57::PointCloudRawT)),
			pointCloudMed(ReconstructorE57::PointCloudMedT::Ptr(new ReconstructorE57::PointCloudMedT)),
			pointCloudRec(ReconstructorE57::PointCloudRecT::Ptr(new  ReconstructorE57::PointCloudRecT)),
			indicesMed(pcl::IndicesPtr(new std::vector<int>)),
			treeRaw(pcl::search::KdTree<ReconstructorE57::PointRawT>::Ptr(new pcl::search::KdTree<ReconstructorE57::PointRawT>())),
			treeMed(pcl::search::KdTree<ReconstructorE57::PointMedT>::Ptr(new pcl::search::KdTree<ReconstructorE57::PointMedT>())),
			treeRec(pcl::search::KdTree<ReconstructorE57::PointRecT>::Ptr(new pcl::search::KdTree<ReconstructorE57::PointRecT>()))
		{
			treeRaw->setInputCloud(pointCloudRaw);
			treeMed->setInputCloud(pointCloudMed);
			treeRec->setInputCloud(pointCloudRec);
		}

		void CloneProcessers(const Global_Reconstruct& global)
		{
			if (global.reconstructorE57->getDownSampler()) downSampler = global.reconstructorE57->getDownSampler()->Clone();
			if (global.reconstructorE57->getOutlierRemover()) outlierRemover = global.reconstructorE57->getOutlierRemover()->Clone();
			if (global.reconstructorE57->getSurfaceProcesser()) surfaceProcesser = global.reconstructorE57->getSurfaceProcesser()->Clone();
			if (global.reconstructorE57->getNormalEstimater()) normalEstimater = global.reconstructorE57->getNormalEstimater()->Clone();
			if (global.reconstructorE57->getAlbedoEstimater()) albedoEstimater = global.reconstructorE57->getAlbedoEstimater()->Clone();
			if (global.reconstructorE57->getSegmenter()) segmenter = global.reconstructorE57->getSegmenter()->Clone();
		}
	};

	int A_ReconstructPointCloud(Global_Reconstruct& global, Query_Reconstruct& query, Data_Reconstruct& data)
	{
		int cerr = global.Check();
		if (cerr == 0)
		{
			std::stringstream strQuery;
			strQuery << query.extMinBB << ", " << query.extMaxBB;
			PRINT_INFO("Start - " + strQuery.str());

			// Init
			data.CloneProcessers(global);

			// Raw
			PRINT_INFO("Load pointCloudRaw");
			pcl::PCLPointCloud2::Ptr blob(new pcl::PCLPointCloud2);
			global.reconstructorE57->getContainerRaw()->queryBoundingBox(query.extMinBB, query.extMaxBB, query.depth, blob);
			pcl::fromPCLPointCloud2(*blob, *data.pointCloudRaw);

			// Med
			if (data.downSampler)
			{
				PRINT_INFO("DownSampling pointCloudMed");
				ReconstructorE57::PointCloudRawT::Ptr temp(new ReconstructorE57::PointCloudRawT);
				data.downSampler->setInputCloud(data.pointCloudRaw);
				data.downSampler->filter(*temp);

				data.pointCloudMed->resize(temp->size());
				for (std::size_t px = 0; px < temp->size(); ++px)
					(*data.pointCloudMed)[px] = (*temp)[px];

				std::stringstream ss;
				ss << "DownSampling pointCloudMed - inSize, outSize: " << data.pointCloudRaw->size() << ", " << data.pointCloudMed->size();
				PRINT_INFO(ss.str());
			}
			else
			{
				data.pointCloudMed->resize(data.pointCloudRaw->size());
				for (std::size_t px = 0; px < data.pointCloudRaw->size(); ++px)
					(*data.pointCloudMed)[px] = (*data.pointCloudRaw)[px];
			}

			// Med Indices
			{
				PRINT_INFO("Extract pointCloudMed indices");

				// Outlier Removal
				std::vector<int> orIndices;
				if (data.outlierRemover)
				{
					PRINT_INFO("Extract pointCloudMed indices - Outlier Removal");
					data.outlierRemover->setInputCloud(data.pointCloudMed);
					data.outlierRemover->filter(orIndices);
					std::sort(orIndices.begin(), orIndices.end());
					std::stringstream ss;
					ss << "Extract pointCloudMed indices - Outlier Removal - inSize, outSize: " << data.pointCloudMed->size() << ", " << orIndices.size();
					PRINT_INFO(ss.str());
				}

				// Crop
				std::vector<int> cropIndices;
				{
					PRINT_INFO("Extract pointCloudMed indices - Crop");

					pcl::CropBox<ReconstructorE57::PointMedT> cb;
					cb.setMin(Eigen::Vector4f(query.minBB.x(), query.minBB.y(), query.minBB.z(), 1.0));
					cb.setMax(Eigen::Vector4f(query.maxBB.x(), query.maxBB.y(), query.maxBB.z(), 1.0));
					cb.setInputCloud(data.pointCloudMed);
					cb.filter(cropIndices);
					std::sort(cropIndices.begin(), cropIndices.end());
					std::stringstream ss;
					ss << "Extract pointCloudMed indices - Crop - inSize, outSize: " << data.pointCloudMed->size() << ", " << cropIndices.size();
					PRINT_INFO(ss.str());
				}

				// Combine
				{
					data.indicesMed->resize(std::min(orIndices.size(), cropIndices.size()));
					std::vector<int>::iterator it = std::set_intersection(
						orIndices.begin(), orIndices.end(),
						cropIndices.begin(), cropIndices.end(),
						data.indicesMed->begin());
					data.indicesMed->resize(it - data.indicesMed->begin());

					std::stringstream ss;
					ss << "Extract pointCloudMed indices - inSize, outSize: " << data.pointCloudMed->size() << ", " << data.indicesMed->size();
					PRINT_INFO(ss.str());
				}
			}

			//
			PRINT_INFO("End - " + strQuery.str());
		}
		return cerr;
	}

	int B_ReconstructPointCloud(Global_Reconstruct& global, Query_Reconstruct& query, Data_Reconstruct& data)
	{
		int cerr = global.Check();
		if (cerr == 0)
		{
			std::stringstream strQuery;
			strQuery << query.minBB << ", " << query.maxBB;
			PRINT_INFO("Start - " + strQuery.str());

			// Estimat Normal
			if (data.normalEstimater)
			{
				if (E57xPCD_CAN_CONTAIN_NORMAL)
				{
					PRINT_INFO("Estimat Normal pointCloudMed");

					data.normalEstimater->setSearchMethod(data.treeMed);
					data.normalEstimater->setSearchSurface(data.pointCloudMed);
					data.normalEstimater->setInputCloud(data.pointCloudMed);
					data.normalEstimater->setIndices(data.indicesMed);
					data.normalEstimater->compute(*data.pointCloudMed);
				}
			}

			// Extract
			// Not using ExtractIndices for not  copy again
			data.pointCloudRec->resize(data.indicesMed->size());
			for (std::size_t px = 0; px < data.indicesMed->size(); ++px)
				(*data.pointCloudRec)[px] = (*data.pointCloudMed)[(*data.indicesMed)[px]];

			PRINT_INFO("End - " + strQuery.str());
		}
		return cerr;
	}

	int C_ReconstructPointCloud(Global_Reconstruct& global, Data_Reconstruct& data)
	{
		int cerr = global.Check();
		if (cerr == 0)
		{
			PRINT_INFO("Start");

			(*global.reconstructorE57->getPointCloudRec()) += (*data.pointCloudRec);

			PRINT_INFO("End");
		}
		return cerr;
	}

	void ReconstructorE57::ReconstructPointCloud()
	{
		//
		pointCloudRec->clear();

		//
		std::vector<Query_Reconstruct> queries;
		ContainerRawT::Iterator it(*containerRaw);
		double outofCoreLeafOverlap = SearchRadius();
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
		Global_Reconstruct global(this);
		AsyncProcess<Global_Reconstruct, Query_Reconstruct, Data_Reconstruct, 1>(global, queries, A_ReconstructPointCloud, B_ReconstructPointCloud, C_ReconstructPointCloud);
	}

	int A_ReconstructAlbedo(Global_Reconstruct& global, Query_Reconstruct& query, Data_Reconstruct& data)
	{
		int cerr = global.Check();
		if (cerr == 0)
		{
			std::stringstream strQuery;
			strQuery << query.extMinBB << ", " << query.extMaxBB;
			PRINT_INFO("Start - " + strQuery.str());

			PRINT_INFO("End");
		}
		return cerr;
	}

	int B_ReconstructAlbedo(Global_Reconstruct& global, Query_Reconstruct& query, Data_Reconstruct& data)
	{
		int cerr = global.Check();
		if (cerr == 0)
		{
			std::stringstream strQuery;
			strQuery << query.extMinBB << ", " << query.extMaxBB;
			PRINT_INFO("Start - " + strQuery.str());

			PRINT_INFO("End");
		}
		return cerr;
	}

	int C_ReconstructAlbedo(Global_Reconstruct& global, Data_Reconstruct& data)
	{
		int cerr = global.Check();
		if (cerr == 0)
		{
			PRINT_INFO("Start");

			PRINT_INFO("End");
		}
		return cerr;
	}

	void ReconstructorE57::ReconstructAlbedo()
	{
		if (!pointCloudRec)
			THROW_EXCEPTION("pointCloud is not set.");


	}
}
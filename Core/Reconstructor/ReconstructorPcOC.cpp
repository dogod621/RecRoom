#include <algorithm>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/impl/extract_indices.hpp>

#include <pcl/filters/crop_box.h>
#include <pcl/filters/impl/crop_box.hpp>

#include "Common/AsyncProcess.h"

#include "ReconstructorPcOC.h"

namespace RecRoom
{
	// Async Reconstruct
	class  AsyncGlobal_Rec : public AsyncGlobal
	{
	public:

		AsyncGlobal_Rec(ReconstructorPcOC* reconstructorPcOC = nullptr)
			: reconstructorPcOC(reconstructorPcOC) {}

		virtual int Check() const
		{
			if (reconstructorPcOC == nullptr) return 1;
			if (!reconstructorPcOC->getScanner()) return 2;
			if (!reconstructorPcOC->getContainerPcNDF()) return 3;
			if (!reconstructorPcOC->getPcMED()) return 4;
			return 0;
		}

	public:
		ReconstructorPcOC* ptrReconstructorPcOC()
		{
			return reconstructorPcOC;
		}

		const ReconstructorPcOC* ptrReconstructorPcOC() const
		{
			return reconstructorPcOC;
		}

	protected:
		ReconstructorPcOC* reconstructorPcOC;
	};

	class  AsyncQuery_Rec : public AsyncQuery<AsyncGlobal_Rec>
	{
	public:
		int index;

		AsyncQuery_Rec(int index = -1) : index(index)
		{}

		virtual int Check(const AsyncGlobal_Rec& global) const
		{
			if (index < 0) return 1;
			if (index >= global.ptrReconstructorPcOC()->getScanner()->getContainerPcRAW()->Size()) return 2;
			return 0;
		}

		virtual std::string Info(const AsyncGlobal_Rec& global) const
		{
			std::stringstream ss;
			ContainerPcRAW::Meta meta = global.ptrReconstructorPcOC()->getScanner()->getContainerPcRAW()->GetMeta(index);
			ss << index << ": " << meta.minAABB << ", " << meta.maxAABB;
			return ss.str();
		}
	};

	struct AsyncData_Rec
	{
		PTR(PcMED) pcRaw;
		PTR(PcMED) pcRec;
		PTR(PcIndex) pcRawIdx;
		PTR(PcIndex) pcRecIdx;
		PTR(AccMED) pcRawAcc;
		PTR(AccMED) pcRecAcc;
		PTR(PcIndex) pcReturnIdx;

		AsyncData_Rec() :
			pcRaw(new PcMED),
			pcRec(new PcMED),
			pcRawIdx(new PcIndex),
			pcRecIdx(new PcIndex),
			pcRawAcc(new KDTreeMED),
			pcRecAcc(new KDTreeMED),
			pcReturnIdx(new PcIndex)
		{
		}
	};

	// Async Reconstruct PointCloud
	int AStep_RecPointCloud(const AsyncGlobal_Rec& global, const AsyncQuery_Rec& query, AsyncData_Rec& data)
	{
		// 
		ContainerPcRAW::Data cData;
		{
			PRINT_INFO("Get pointCloud from container - Start");

			cData = global.ptrReconstructorPcOC()->getScanner()->getContainerPcRAW()->GetData(query.index);
			data.pcRaw = cData.pcMED;
			data.pcRawIdx = cData.pcIndex;

			std::stringstream ss;
			ss << "Get pointCloud from container - End - pcSize: " << data.pcRaw->size() << ", idxSize: " << data.pcRawIdx->size();
			PRINT_INFO(ss.str());
		}

		{
			PRINT_INFO("Build pcRawAcc - Start");
			data.pcRawAcc->setInputCloud(data.pcRaw);
			PRINT_INFO("Build pcRawAcc - End");
		}

		// 
		if (global.ptrReconstructorPcOC()->getDownSampler())
		{
			{
				PRINT_INFO("DownSampling - Start");

				global.ptrReconstructorPcOC()->getDownSampler()->Process(data.pcRawAcc, data.pcRaw, nullptr, *data.pcRec);

				std::stringstream ss;
				ss << "DownSampling - End - inSize: " << data.pcRaw->size() << ", outSize:" << data.pcRec->size();
				PRINT_INFO(ss.str());
			}

			{
				PRINT_INFO("Build pcRecAcc - Start");
				data.pcRecAcc->setInputCloud(data.pcRec);
				PRINT_INFO("Build pcRecAcc - End");
			}

			{
				PRINT_INFO("Extract indices - Start");

				pcl::CropBox<PointMED> cb;
				cb.setMin(Eigen::Vector4f(cData.minAABB.x(), cData.minAABB.y(), cData.minAABB.z(), 1.0));
				cb.setMax(Eigen::Vector4f(cData.maxAABB.x(), cData.maxAABB.y(), cData.maxAABB.z(), 1.0));
				cb.setInputCloud(data.pcRec);
				cb.filter(*data.pcRecIdx);
				
				std::stringstream ss;
				ss << "Extract indices - End - pcSize: " << data.pcRec->size() << ", idxSize: " << data.pcRecIdx->size();
				PRINT_INFO(ss.str());
			}
		}
		else
		{
			PRINT_WARNING("downSampler is not set, ignore it");
			data.pcRec = data.pcRaw;
			data.pcRecIdx = data.pcRawIdx;
			data.pcRecAcc = data.pcRawAcc;
		}

		//
		if (global.ptrReconstructorPcOC()->getOutlierRemover())
		{
			PRINT_INFO("Outlier Removal - Start");


			PcIndex orIndices;
			global.ptrReconstructorPcOC()->getOutlierRemover()->Process(data.pcRecAcc, data.pcRec, nullptr, orIndices);
			
			// Combine
			std::sort(orIndices.begin(), orIndices.end());
			std::sort(data.pcRecIdx->begin(), data.pcRecIdx->end());
			PcIndex indices = *data.pcRecIdx;

			data.pcRecIdx->resize(std::min(orIndices.size(), indices.size()));
			std::vector<int>::iterator it = std::set_intersection(
				indices.begin(), indices.end(),
				orIndices.begin(), orIndices.end(),
				data.pcRecIdx->begin());
			data.pcRecIdx->resize(it - data.pcRecIdx->begin());

			//
			std::stringstream ss;
			ss << "Outlier Removal - End - pcSize: " << indices.size() << ", idxSize: " << data.pcRecIdx->size();
			PRINT_INFO(ss.str());
		}
		else
		{
			PRINT_WARNING("outlierRemover is not set, ignore it");
		}

		return 0;
	}

	int BStep_RecPointCloud(const AsyncGlobal_Rec& global, const AsyncQuery_Rec& query, AsyncData_Rec& data)
	{
#ifdef INPUT_PERPOINT_NORMAL
		THROW_EXCEPTION("Not done, using scane normal is a feature in to-do list.");
		return 1;
#elif defined OUTPUT_PERPOINT_NORMAL
		if (global.ptrReconstructorPcOC()->getNormalEstimator())
		{
			PRINT_INFO("Estimat Normal - Start");

			global.ptrReconstructorPcOC()->getNormalEstimator()->ProcessInOut(
				data.pcRecAcc, data.pcRec, data.pcRecIdx);

			PRINT_INFO("Estimat Normal - End");
		}
		else
		{
			PRINT_WARNING("normalEstimator is not set, ignore it");
		}
		return 0;
#else
		PRINT_WARNING("None of INPUT_PERPOINT_NORMAL or OUTPUT_PERPOINT_NORMAL is set, ignore it");
		return 0;
#endif
	}

	int CStep_RecPointCloud(AsyncGlobal_Rec& global, const AsyncQuery_Rec& query, const AsyncData_Rec& data)
	{
		PcMED temp;
		{
			PRINT_INFO("Extract - Start");

			pcl::ExtractIndices<PointMED> extract;
			extract.setInputCloud(data.pcRec);
			extract.setIndices(data.pcRecIdx);
			extract.setNegative(false);
			extract.filter(temp);

			PRINT_INFO("Extract - End - Size: " + std::to_string(temp.size()));
		}

		PRINT_INFO("Merge - Start");

		(*global.ptrReconstructorPcOC()->getPcMED()) += temp;

		PRINT_INFO("Merge - End - pcSize: " + std::to_string(global.ptrReconstructorPcOC()->getPcMED()->size()));


		// Som wired noise, debugging...
		if (global.ptrReconstructorPcOC()->getDownSampler())
		{
			PTR(PcMED)pcMED2(new PcMED);
			PTR(AccMED)pcMEDAcc(new KDTreeMED);
			pcMEDAcc->setInputCloud(global.ptrReconstructorPcOC()->getPcMED());

			{
				PRINT_INFO("DownSampling - Start");

				global.ptrReconstructorPcOC()->getDownSampler()->Process(pcMEDAcc, global.ptrReconstructorPcOC()->getPcMED(), nullptr, *pcMED2);

				std::stringstream ss;
				ss << "DownSampling - End - inSize: " << global.ptrReconstructorPcOC()->getPcMED()->size() << ", outSize:" << pcMED2->size();
				PRINT_INFO(ss.str());
			}

			(*global.ptrReconstructorPcOC()->getPcMED()) = (*pcMED2);
		}

		//
		return 0;
	}

	void ReconstructorPcOC::RecPointCloud()
	{
		AsyncGlobal_Rec global(this);

		std::vector<AsyncQuery_Rec> queries(scanner->getContainerPcRAW()->Size());
		for (std::size_t i = 0; i < queries.size(); ++i)
			queries[i].index = i;

		AsyncProcess<AsyncGlobal_Rec, AsyncQuery_Rec, AsyncData_Rec>(
			global, queries,
			AStep_RecPointCloud, BStep_RecPointCloud, CStep_RecPointCloud,
			asyncSize);
	}

	// Async Reconstruct Attribute
	int AStep_RecPcAtt(const AsyncGlobal_Rec& global, const AsyncQuery_Rec& query, AsyncData_Rec& data)
	{
		// 
		ContainerPcRAW::Data cData;
		{
			PRINT_INFO("Get pointCloud from container - Start");

			cData = global.ptrReconstructorPcOC()->getScanner()->getContainerPcRAW()->GetData(query.index);
			data.pcRaw = cData.pcMED;
			data.pcRawIdx = cData.pcIndex;

			std::stringstream ss;
			ss << "Get pointCloud from container - End - pcSize: " << data.pcRaw->size() << ", idxSize: " << data.pcRawIdx->size();
			PRINT_INFO(ss.str());
		}

		{
			PRINT_INFO("Build pcRawAcc - Start");
			data.pcRawAcc->setInputCloud(data.pcRaw);
			PRINT_INFO("Build pcRawAcc - End");
		}

		// 
		{
			PRINT_INFO("Extract return indices - Start");

			pcl::CropBox<PointMED> cb;
			cb.setMin(Eigen::Vector4f(cData.minAABB.x(), cData.minAABB.y(), cData.minAABB.z(), 1.0));
			cb.setMax(Eigen::Vector4f(cData.maxAABB.x(), cData.maxAABB.y(), cData.maxAABB.z(), 1.0));
			cb.setInputCloud(global.ptrReconstructorPcOC()->getPcMED());
			cb.filter(*data.pcReturnIdx);

			std::stringstream ss;
			ss << "Extract return indices - End - pcSize: " << global.ptrReconstructorPcOC()->getPcMED()->size() << ", idxSize: " << data.pcReturnIdx->size();
			PRINT_INFO(ss.str());
		}

		// 
		{
			PRINT_INFO("Extract pointCloud from pcMED - Start");

			pcl::CropBox<PointMED> cb;
			cb.setMin(Eigen::Vector4f(cData.extMinAABB.x(), cData.extMinAABB.y(), cData.extMinAABB.z(), 1.0));
			cb.setMax(Eigen::Vector4f(cData.extMaxAABB.x(), cData.extMaxAABB.y(), cData.extMaxAABB.z(), 1.0));
			cb.setInputCloud(global.ptrReconstructorPcOC()->getPcMED());
			cb.filter(*data.pcRec);

			std::stringstream ss;
			ss << "Extract pointCloud from pcMED - End - inSize: " << global.ptrReconstructorPcOC()->getPcMED()->size() << ", outSize:" << data.pcRec->size();
			PRINT_INFO(ss.str());

		}

		{
			PRINT_INFO("Build pcRecAcc - Start");
			data.pcRecAcc->setInputCloud(data.pcRec);
			PRINT_INFO("Build pcRecAcc - End");
		}

		{
			PRINT_INFO("Extract indices - Start");

			pcl::CropBox<PointMED> cb;
			cb.setMin(Eigen::Vector4f(cData.minAABB.x(), cData.minAABB.y(), cData.minAABB.z(), 1.0));
			cb.setMax(Eigen::Vector4f(cData.maxAABB.x(), cData.maxAABB.y(), cData.maxAABB.z(), 1.0));
			cb.setInputCloud(data.pcRec);
			cb.filter(*data.pcRecIdx);

			std::stringstream ss;
			ss << "Extract indices - End - inSize: " << data.pcRec->size() << ", outSize:" << data.pcRecIdx->size();
			PRINT_INFO(ss.str());

			// Check
			if (data.pcRecIdx->size() != data.pcReturnIdx->size())
				return 1;
		}

		{
			PRINT_INFO("Upsampling Attribute - Start");

			global.ptrReconstructorPcOC()->getInterpolator()->ProcessInOut(data.pcRecAcc, data.pcRaw, nullptr);

			PRINT_INFO("Upsampling Attribute - End");
		}

		return 0;
	}

	int CStep_RecPcAtt(AsyncGlobal_Rec& global, const AsyncQuery_Rec& query, const AsyncData_Rec& data)
	{
		// Check
		if (data.pcRecIdx->size() != data.pcReturnIdx->size())
			return 1;

		PRINT_INFO("Update Attribute - Start");

		PcMED& srcPC = *data.pcRec;
		PcMED& tarPC = *global.ptrReconstructorPcOC()->getPcMED();
		PcIndex& srcIdx = *data.pcRecIdx;
		PcIndex& tarIdx = *data.pcReturnIdx;
		for (std::size_t px = 0; px < srcIdx.size(); ++px)
			tarPC[tarIdx[px]] = srcPC[srcIdx[px]];

		PRINT_INFO("Update Attribute - End");

		return 0;
	}

	// Async Reconstruct Attribute - NDF
	int BStep_RecPcAlbedo_NDF(const AsyncGlobal_Rec& global, const AsyncQuery_Rec& query, AsyncData_Rec& data)
	{
		PRINT_INFO("Estimat NDF - Start");

		global.ptrReconstructorPcOC()->getNDFEstimator()->ProcessInOut(
			data.pcRawAcc, data.pcRec, data.pcRecIdx);

		PRINT_INFO("Estimat NDF - End");

		return 0;
	}

	// Async Reconstruct Attribute - Albedo
	int BStep_RecPcAlbedo_ALBEDO(const AsyncGlobal_Rec& global, const AsyncQuery_Rec& query, AsyncData_Rec& data)
	{
		PRINT_INFO("Estimat Albedo - Start");

		global.ptrReconstructorPcOC()->getAlbedoEstimator()->ProcessInOut(
			data.pcRawAcc, data.pcRec, data.pcRecIdx);

		PRINT_INFO("Estimat Albedo - End");

		return 0;
	}

	// Async Reconstruct Attribute - NDF
	int BStep_RecSegNDF(const AsyncGlobal_Rec& global, const AsyncQuery_Rec& query, AsyncData_Rec& data)
	{
		return 0;
	}

	int CStep_RecSegNDF(AsyncGlobal_Rec& global, const AsyncQuery_Rec& query, const AsyncData_Rec& data)
	{
		const double cutFalloff = 0.33; // 

#ifdef PERPOINT_LABEL
		PTR(PcNDF) pcNDF (new PcNDF);
		pcNDF->reserve(data.pcRawIdx->size());
		for (PcIndex::const_iterator it = data.pcRawIdx->begin(); it != data.pcRawIdx->end(); ++it)
		{
			PointMED& pRaw = (*data.pcRaw)[*it];
			if (pRaw.HasLabel())
			{
				ScanLaser scanLaser;
				if (global.ptrReconstructorPcOC()->getScanner()->ToScanLaser(pRaw, scanLaser))
				{
					Eigen::Vector3d hafway = scanLaser.incidentDirection + scanLaser.reflectedDirection;
					double hafwayNorm = hafway.norm();
					if (hafwayNorm > Common::eps)
					{
						hafway /= hafwayNorm;
						if (scanLaser.beamFalloff > cutFalloff)
						{
							Eigen::Vector3d tanHafway(
								scanLaser.hitTangent.dot(hafway),
								scanLaser.hitBitangent.dot(hafway),
								scanLaser.hitNormal.dot(hafway));
							pcNDF->push_back(PointNDF(hafway.x(), hafway.y(), hafway.z(), pRaw.label, scanLaser.intensity / scanLaser.beamFalloff));
						}
					}
				}
			}
			else
				PRINT_WARNING("!pRaw.HasSegLabel(), ignore");
		}
		global.ptrReconstructorPcOC()->getContainerPcNDF()->Merge(pcNDF);
#endif
		return 0;
	}

	void ReconstructorPcOC::RecPcMaterial_NDF()
	{
		AsyncGlobal_Rec global(this);

		std::vector<AsyncQuery_Rec> queries(scanner->getContainerPcRAW()->Size());
		for (std::size_t i = 0; i < queries.size(); ++i)
			queries[i].index = i;

		AsyncProcess<AsyncGlobal_Rec, AsyncQuery_Rec, AsyncData_Rec>(
			global, queries,
			AStep_RecPcAtt, BStep_RecPcAlbedo_NDF, CStep_RecPcAtt,
			asyncSize);
	}

	void ReconstructorPcOC::RecPcMaterial_ALBEDO()
	{
		AsyncGlobal_Rec global(this);

		std::vector<AsyncQuery_Rec> queries(scanner->getContainerPcRAW()->Size());
		for (std::size_t i = 0; i < queries.size(); ++i)
			queries[i].index = i;

		AsyncProcess<AsyncGlobal_Rec, AsyncQuery_Rec, AsyncData_Rec>(
			global, queries,
			AStep_RecPcAtt, BStep_RecPcAlbedo_ALBEDO, CStep_RecPcAtt,
			asyncSize);
	}

	void ReconstructorPcOC::RecSegMaterial()
	{
		AsyncGlobal_Rec global(this);

		std::vector<AsyncQuery_Rec> queries(scanner->getContainerPcRAW()->Size());
		for (std::size_t i = 0; i < queries.size(); ++i)
			queries[i].index = i;

		AsyncProcess<AsyncGlobal_Rec, AsyncQuery_Rec, AsyncData_Rec>(
			global, queries,
			AStep_RecPcAtt, BStep_RecSegNDF, CStep_RecSegNDF,
			asyncSize);
	}
}
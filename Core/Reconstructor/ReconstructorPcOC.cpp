#include <algorithm>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/impl/extract_indices.hpp>

#include "Common/AsyncProcess.h"
#include "Common/VoxelGrid.h"
#include "Filter/FilterPcAABB.h"

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

		AsyncData_Rec() :
			pcRaw(new PcMED),
			pcRec(new PcMED),
			pcRawIdx(new PcIndex),
			pcRecIdx(new PcIndex),
			pcRawAcc(nullptr),
			pcRecAcc(new KDTreeMED)
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
			PRINT_INFO("Build pcRawAcc - Start - vnn:" + std::to_string(global.ptrReconstructorPcOC()->getUseVNN()));
			if (global.ptrReconstructorPcOC()->getUseVNN())
			{
				float res = global.ptrReconstructorPcOC()->getResVNN();
				data.pcRawAcc = PTR(VNN<PointMED>)(new VNN<PointMED> (Eigen::Vector3d(res, res, res), cData.minAABB, cData.maxAABB));
			}
			else
			{
				data.pcRawAcc = PTR(AccMED)(new KDTreeMED);
			}
			data.pcRawAcc->setInputCloud(data.pcRaw);
			PRINT_INFO("Build pcRawAcc - End");
		}

		// 
		if (global.ptrReconstructorPcOC()->getDownSampler())
		{
			global.ptrReconstructorPcOC()->getDownSampler()->Process(data.pcRawAcc, data.pcRaw, nullptr, *data.pcRec);
			
			PRINT_INFO("Build pcRecAcc - Start");
			data.pcRecAcc->setInputCloud(data.pcRec);
			PRINT_INFO("Build pcRecAcc - End");
			
			FilterPcAABB<PointMED> cb(cData.minAABB, cData.maxAABB);
			cb.Process(nullptr, data.pcRec, nullptr, *data.pcRecIdx);
		}
		else
		{
			data.pcRec = data.pcRaw;
			data.pcRecIdx = data.pcRawIdx;
			data.pcRecAcc = data.pcRawAcc;
		}

		//
		if (global.ptrReconstructorPcOC()->getOutlierRemover())
		{
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
		}
		return 0;
	}

	int BStep_RecPointCloud(const AsyncGlobal_Rec& global, const AsyncQuery_Rec& query, AsyncData_Rec& data)
	{
		return 0;
	}

	int CStep_RecPointCloud(AsyncGlobal_Rec& global, const AsyncQuery_Rec& query, const AsyncData_Rec& data)
	{
		PRINT_INFO("Merge - Start");

		PcMED temp;
		{
			pcl::ExtractIndices<PointMED> extract;
			extract.setInputCloud(data.pcRec);
			extract.setIndices(data.pcRecIdx);
			extract.setNegative(false);
			extract.filter(temp);
		}

		(*global.ptrReconstructorPcOC()->getPcMED()) += temp;

		PRINT_INFO("Merge - End - pcSize: " + std::to_string(global.ptrReconstructorPcOC()->getPcMED()->size()));

		return 0;
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
			PRINT_INFO("Build pcRawAcc - Start - vnn:" + std::to_string(global.ptrReconstructorPcOC()->getUseVNN()));
			if (global.ptrReconstructorPcOC()->getUseVNN())
			{
				float res = global.ptrReconstructorPcOC()->getResVNN();
				data.pcRawAcc = PTR(VNN<PointMED>)(new VNN<PointMED>(Eigen::Vector3d(res, res, res), cData.minAABB, cData.maxAABB));
			}
			else
			{
				data.pcRawAcc = PTR(AccMED)(new KDTreeMED);
			}
			data.pcRawAcc->setInputCloud(data.pcRaw);
			PRINT_INFO("Build pcRawAcc - End");
		}

		// 
		{
			data.pcRec = global.ptrReconstructorPcOC()->getPcMED();

			FilterPcAABB<PointMED> cb(cData.minAABB, cData.maxAABB);
			cb.Process(nullptr, data.pcRec, nullptr, *data.pcRecIdx);

			PTR(PcIndex) cbIdxEXT (new PcIndex);
			FilterPcAABB<PointMED> cbEXT (cData.extMinAABB, cData.extMaxAABB);
			cbEXT.Process(nullptr, data.pcRec, nullptr, *cbIdxEXT);

			PRINT_INFO("Build pcRecAcc - Start");
			data.pcRecAcc->setInputCloud(data.pcRec, cbIdxEXT);
			PRINT_INFO("Build pcRecAcc - End");
		}

		return 0;
	}

	int CStep_RecPcAtt(AsyncGlobal_Rec& global, const AsyncQuery_Rec& query, const AsyncData_Rec& data)
	{
		// Check
		return 0;
	}

	// Async Reconstruct Attribute - Normal
	int BStep_RecPcNormal(const AsyncGlobal_Rec& global, const AsyncQuery_Rec& query, AsyncData_Rec& data)
	{
		{
			global.ptrReconstructorPcOC()->getNormalEstimator()->ProcessInOut(
				data.pcRawAcc, data.pcRec, data.pcRecIdx);
		}
		return 0;
	}

	// Async Reconstruct Attribute - Albedo
	int BStep_RecPcAlbedo(const AsyncGlobal_Rec& global, const AsyncQuery_Rec& query, AsyncData_Rec& data)
	{
		{
			PcMED temp;

			global.ptrReconstructorPcOC()->getInterpolator()->Process(data.pcRecAcc, data.pcRaw, nullptr, temp);

			for (std::size_t px = 0; px < data.pcRaw->size(); ++px)
			{
				PointMED& tarP = (*data.pcRaw)[px];
				PointMED& srcP = temp[px];

				tarP.normal_x = srcP.normal_x;
				tarP.normal_y = srcP.normal_y;
				tarP.normal_z = srcP.normal_z;
				tarP.curvature = srcP.curvature;
				tarP.label = srcP.label;
			}
		}

		{
			global.ptrReconstructorPcOC()->getAlbedoEstimator()->ProcessInOut(
				data.pcRawAcc, data.pcRec, data.pcRecIdx);
		}
		return 0;
	}

	// Async Reconstruct Attribute - Sharpness
	int BStep_RecPcSharpness(const AsyncGlobal_Rec& global, const AsyncQuery_Rec& query, AsyncData_Rec& data)
	{
		{
			PcMED temp;

			global.ptrReconstructorPcOC()->getInterpolator()->Process(data.pcRecAcc, data.pcRaw, nullptr, temp);

			for (std::size_t px = 0; px < data.pcRaw->size(); ++px)
			{
				PointMED& tarP = (*data.pcRaw)[px];
				PointMED& srcP = temp[px];

				tarP.intensity = srcP.intensity;
				tarP.normal_x = srcP.normal_x;
				tarP.normal_y = srcP.normal_y;
				tarP.normal_z = srcP.normal_z;
				tarP.curvature = srcP.curvature;
				tarP.label = srcP.label;
			}
		}

		{
			global.ptrReconstructorPcOC()->getSharpnessEstimator()->ProcessInOut(
				data.pcRawAcc, data.pcRec, data.pcRecIdx);
		}
		return 0;
	}

	// Async Reconstruct Attribute - NDF
	int BStep_RecSegNDF(const AsyncGlobal_Rec& global, const AsyncQuery_Rec& query, AsyncData_Rec& data)
	{
		return 0;
	}

	int CStep_RecSegNDF(AsyncGlobal_Rec& global, const AsyncQuery_Rec& query, const AsyncData_Rec& data)
	{
		const float cutFalloff = 0.33f; // 

		{
			PcMED temp;

			global.ptrReconstructorPcOC()->getInterpolator()->Process(data.pcRecAcc, data.pcRaw, nullptr, temp);

			for (std::size_t px = 0; px < data.pcRaw->size(); ++px)
			{
				PointMED& tarP = (*data.pcRaw)[px];
				PointMED& srcP = temp[px];

				tarP.normal_x = srcP.normal_x;
				tarP.normal_y = srcP.normal_y;
				tarP.normal_z = srcP.normal_z;
				tarP.curvature = srcP.curvature;
				tarP.label = srcP.label;
			}
		}

#ifdef PERPOINT_NORMAL
#ifdef PERPOINT_LABEL
		PTR(PcNDF) pcNDF (new PcNDF);
		pcNDF->reserve(data.pcRawIdx->size());
		for (PcIndex::const_iterator it = data.pcRawIdx->begin(); it != data.pcRawIdx->end(); ++it)
		{
			PointMED& pRaw = (*data.pcRaw)[*it];
			if (pRaw.HasLabel())
			{
				Eigen::Vector3f hitNormal(pRaw.normal_x, pRaw.normal_y, pRaw.normal_z);
				Eigen::Vector3f hitTangent;
				Eigen::Vector3f hitBitangent;
				if (Common::GenFrame(hitNormal, hitTangent, hitBitangent))
				{
					ScanLaser scanLaser;
					if (global.ptrReconstructorPcOC()->getScanner()->ToScanLaser(pRaw, scanLaser))
					{
						Eigen::Vector3f hafway = scanLaser.incidentDirection + scanLaser.reflectedDirection;
						float hafwayNorm = hafway.norm();
						if (hafwayNorm > std::numeric_limits<float>::epsilon())
						{
							hafway /= hafwayNorm;
							if (scanLaser.beamFalloff > cutFalloff)
							{
								Eigen::Vector3f tanHafway(
									hitTangent.dot(hafway),
									hitBitangent.dot(hafway),
									hitNormal.dot(hafway));
								pcNDF->push_back(PointNDF(hafway.x(), hafway.y(), hafway.z(), pRaw.label, scanLaser.intensity / scanLaser.beamFalloff));
							}
						}
					}
				}
			}
			else
				PRINT_WARNING("!pRaw.HasSegLabel(), ignore");
		}
		global.ptrReconstructorPcOC()->getContainerPcNDF()->Merge(pcNDF);
#endif
#endif
		return 0;
	}

	void ReconstructorPcOC::ImplementRecPointCloud()
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

	void ReconstructorPcOC::ImplementRecPcNormal()
	{
		AsyncGlobal_Rec global(this);

		std::vector<AsyncQuery_Rec> queries(scanner->getContainerPcRAW()->Size());
		for (std::size_t i = 0; i < queries.size(); ++i)
			queries[i].index = i;

		AsyncProcess<AsyncGlobal_Rec, AsyncQuery_Rec, AsyncData_Rec>(
			global, queries,
			AStep_RecPcAtt, BStep_RecPcNormal, CStep_RecPcAtt,
			asyncSize);
	}

	void ReconstructorPcOC::ImplementRecPcAlbedo()
	{
		AsyncGlobal_Rec global(this);

		std::vector<AsyncQuery_Rec> queries(scanner->getContainerPcRAW()->Size());
		for (std::size_t i = 0; i < queries.size(); ++i)
			queries[i].index = i;

		AsyncProcess<AsyncGlobal_Rec, AsyncQuery_Rec, AsyncData_Rec>(
			global, queries,
			AStep_RecPcAtt, BStep_RecPcAlbedo, CStep_RecPcAtt,
			asyncSize);
	}

	void ReconstructorPcOC::ImplementRecPcSharpness()
	{
		AsyncGlobal_Rec global(this);

		std::vector<AsyncQuery_Rec> queries(scanner->getContainerPcRAW()->Size());
		for (std::size_t i = 0; i < queries.size(); ++i)
			queries[i].index = i;

		AsyncProcess<AsyncGlobal_Rec, AsyncQuery_Rec, AsyncData_Rec>(
			global, queries,
			AStep_RecPcAtt, BStep_RecPcSharpness, CStep_RecPcAtt,
			asyncSize);
	}

	void ReconstructorPcOC::ImplementRecSegMaterial()
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
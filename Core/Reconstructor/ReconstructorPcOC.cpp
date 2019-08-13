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
		/*if (global.ptrReconstructorPcOC()->getOutlierRemover())
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
		}*/
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

			PTR(PcIndex) cbIdxEXT(new PcIndex);
			FilterPcAABB<PointMED> cbEXT(cData.extMinAABB, cData.extMaxAABB);
			cbEXT.Process(nullptr, data.pcRec, nullptr, *cbIdxEXT);

			if (!data.pcRecIdx->empty())
			{
				PRINT_INFO("Build pcRecAcc - Start");
				data.pcRecAcc->setInputCloud(data.pcRec, cbIdxEXT);
				PRINT_INFO("Build pcRecAcc - End");
			}
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
		if (!data.pcRecIdx->empty())
		{
			global.ptrReconstructorPcOC()->getNormalEstimator()->ProcessInOut(
				data.pcRawAcc, data.pcRec, data.pcRecIdx);
		}
		return 0;
	}

	// Async Reconstruct Attribute - Diffuse
	int BStep_RecPcDiffuse(const AsyncGlobal_Rec& global, const AsyncQuery_Rec& query, AsyncData_Rec& data)
	{
		if (!data.pcRecIdx->empty())
		{
			PcMED temp;

			global.ptrReconstructorPcOC()->getFieldInterpolator()->Process(data.pcRecAcc, data.pcRaw, nullptr, temp);

			for (std::size_t px = 0; px < data.pcRaw->size(); ++px)
			{
				PointMED& tarP = (*data.pcRaw)[px];
				PointMED& srcP = temp[px];

				tarP.normal_x = srcP.normal_x;
				tarP.normal_y = srcP.normal_y;
				tarP.normal_z = srcP.normal_z;
				tarP.curvature = srcP.curvature;
			}

			global.ptrReconstructorPcOC()->getDiffuseEstimator()->ProcessInOut(
				data.pcRawAcc, data.pcRec, data.pcRecIdx);
		}
		return 0;
	}

	// Async Reconstruct Attribute - Specular
	int BStep_RecPcSpecular(const AsyncGlobal_Rec& global, const AsyncQuery_Rec& query, AsyncData_Rec& data)
	{
		if (!data.pcRecIdx->empty())
		{
			PcMED temp;

			global.ptrReconstructorPcOC()->getFieldInterpolator()->Process(data.pcRecAcc, data.pcRaw, nullptr, temp);

			for (std::size_t px = 0; px < data.pcRaw->size(); ++px)
			{
				PointMED& tarP = (*data.pcRaw)[px];
				PointMED& srcP = temp[px];

				tarP.normal_x = srcP.normal_x;
				tarP.normal_y = srcP.normal_y;
				tarP.normal_z = srcP.normal_z;
				tarP.curvature = srcP.curvature;

				tarP.diffuseAlbedo = srcP.diffuseAlbedo;
			}

			global.ptrReconstructorPcOC()->getSpecularEstimator()->ProcessInOut(
				data.pcRawAcc, data.pcRec, data.pcRecIdx);
		}
		return 0;
	}

	// Async Reconstruct Attribute - Refine Specular
	int BStep_RecPcRefineSpecular(const AsyncGlobal_Rec& global, const AsyncQuery_Rec& query, AsyncData_Rec& data)
	{
		if (!data.pcRecIdx->empty())
		{
			PcMED temp;

			global.ptrReconstructorPcOC()->getFieldInterpolator()->Process(data.pcRecAcc, data.pcRaw, nullptr, temp);

			for (std::size_t px = 0; px < data.pcRaw->size(); ++px)
			{
				PointMED& tarP = (*data.pcRaw)[px];
				PointMED& srcP = temp[px];

				tarP.normal_x = srcP.normal_x;
				tarP.normal_y = srcP.normal_y;
				tarP.normal_z = srcP.normal_z;
				tarP.curvature = srcP.curvature;

				tarP.diffuseAlbedo = srcP.diffuseAlbedo;
				tarP.specularAlbedo = srcP.specularAlbedo;
				tarP.specularSharpness = srcP.specularSharpness;
			}

			global.ptrReconstructorPcOC()->getRefineSpecularEstimator()->ProcessInOut(
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

		if (!data.pcRecIdx->empty())
		{
			PcMED temp;

			global.ptrReconstructorPcOC()->getFieldInterpolator()->Process(data.pcRecAcc, data.pcRaw, data.pcRawIdx, temp);

			for (std::size_t idx = 0; idx < data.pcRawIdx->size(); ++idx)
			{
				PointMED& tarP = (*data.pcRaw)[(*data.pcRawIdx)[idx]];
				PointMED& srcP = temp[idx];

				tarP.normal_x = srcP.normal_x;
				tarP.normal_y = srcP.normal_y;
				tarP.normal_z = srcP.normal_z;
				tarP.curvature = srcP.curvature;

				tarP.diffuseAlbedo = srcP.diffuseAlbedo;
				tarP.specularAlbedo = srcP.specularAlbedo;
				tarP.specularSharpness = srcP.specularSharpness;

				tarP.softLabelStart = srcP.softLabelStart;
				tarP.softLabelEnd = srcP.softLabelEnd;
			}

			PTR(PcNDF) pcNDF(new PcNDF);
			pcNDF->reserve(data.pcRawIdx->size());
			for (PcIndex::const_iterator it = data.pcRawIdx->begin(); it != data.pcRawIdx->end(); ++it)
			{
				PointMED& pRaw = (*data.pcRaw)[*it];
				if (pcl::isFinite(pRaw) && pRaw.HasSerialNumber())
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
									float intensity = scanLaser.intensity / scanLaser.beamFalloff;
									float diffuseValue = tanHafway.z() * pRaw.diffuseAlbedo;

									if (intensity > diffuseValue)
									{
										for (uint32_t sx = pRaw.softLabelStart; sx < pRaw.softLabelEnd; sx++)
											pcNDF->push_back(PointNDF(tanHafway.x(), tanHafway.y(), tanHafway.z(), 
												(*global.ptrReconstructorPcOC()->getPcSoftLabel())[sx].label,
												pRaw.serialNumber, (intensity - diffuseValue),
												(*global.ptrReconstructorPcOC()->getPcSoftLabel())[sx].weight));
									}
								}
							}
						}
					}
				}
			}
			global.ptrReconstructorPcOC()->getContainerPcNDF()->Merge(pcNDF);
		}
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

		// NAN fill
		{
			PTR(PcIndex) validFilter(new PcIndex);
			PTR(PcIndex) inValidFilter(new PcIndex);
			validFilter->reserve(pcMED->size());
			inValidFilter->reserve(pcMED->size());
			for (int px = 0; px < pcMED->size(); px++)
			{
				if (pcl_isfinite((*pcMED)[px].x) && pcl_isfinite((*pcMED)[px].y) && pcl_isfinite((*pcMED)[px].z))
					validFilter->push_back(px);
				else
					inValidFilter->push_back(px);
			}

			if ((validFilter->size() > 0) && (inValidFilter->size() > 0))
			{
				PTR(PcMED) pcMED2(new PcMED);
				pcMED2->reserve(pcMED->size());

				for (std::size_t idx = 0; idx < validFilter->size(); ++idx)
				{
					pcMED2->push_back((*pcMED)[(*validFilter)[idx]]);
				}
				pcMED = pcMED2;
			}
		}
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

		// NAN fill
		{
			PTR(PcIndex) validFilter(new PcIndex);
			PTR(PcIndex) inValidFilter(new PcIndex);
			validFilter->reserve(pcMED->size());
			inValidFilter->reserve(pcMED->size());
			for (int px = 0; px < pcMED->size(); px++)
			{
				if (normalEstimator->InputPointValid((*pcMED)[px]) && normalEstimator->OutputPointValid((*pcMED)[px]))
					validFilter->push_back(px);
				else
					inValidFilter->push_back(px);
			}

			if ((validFilter->size() > 0) && (inValidFilter->size() > 0))
			{
				PTR(AccMED) validAcc(new KDTreeMED);
				validAcc->setInputCloud(pcMED, validFilter);

				PcMED temp;

				fieldInterpolator->Process(validAcc, pcMED, inValidFilter, temp);

				for (std::size_t idx = 0; idx < inValidFilter->size(); ++idx)
				{
					PointMED& tarP = (*pcMED)[(*inValidFilter)[idx]];
					PointMED& srcP = temp[idx];

					tarP.normal_x = srcP.normal_x;
					tarP.normal_y = srcP.normal_y;
					tarP.normal_z = srcP.normal_z;
					tarP.curvature = srcP.curvature;
				}
			}
		}
	}

	void ReconstructorPcOC::ImplementRecPcDiffuse()
	{
		AsyncGlobal_Rec global(this);

		std::vector<AsyncQuery_Rec> queries(scanner->getContainerPcRAW()->Size());
		for (std::size_t i = 0; i < queries.size(); ++i)
			queries[i].index = i;

		AsyncProcess<AsyncGlobal_Rec, AsyncQuery_Rec, AsyncData_Rec>(
			global, queries,
			AStep_RecPcAtt, BStep_RecPcDiffuse, CStep_RecPcAtt,
			asyncSize);

		// NAN fill
		{
			PTR(PcIndex) validFilter(new PcIndex);
			PTR(PcIndex) inValidFilter(new PcIndex);
			validFilter->reserve(pcMED->size());
			inValidFilter->reserve(pcMED->size());
			for (int px = 0; px < pcMED->size(); px++)
			{
				if (diffuseEstimator->InputPointValid((*pcMED)[px]) && diffuseEstimator->OutputPointValid((*pcMED)[px]))
					validFilter->push_back(px);
				else
					inValidFilter->push_back(px);
			}

			if ((validFilter->size() > 0) && (inValidFilter->size() > 0))
			{
				PTR(AccMED) validAcc(new KDTreeMED);
				validAcc->setInputCloud(pcMED, validFilter);

				PcMED temp;

				fieldInterpolator->Process(validAcc, pcMED, inValidFilter, temp);

				for (std::size_t idx = 0; idx < inValidFilter->size(); ++idx)
				{
					PointMED& tarP = (*pcMED)[(*inValidFilter)[idx]];
					PointMED& srcP = temp[idx];

					tarP.r = srcP.r;
					tarP.g = srcP.g;
					tarP.b = srcP.b;

					tarP.normal_x = srcP.normal_x;
					tarP.normal_y = srcP.normal_y;
					tarP.normal_z = srcP.normal_z;
					tarP.curvature = srcP.curvature;

					tarP.diffuseAlbedo = srcP.diffuseAlbedo;
				}
			}
		}
	}

	void ReconstructorPcOC::ImplementRecPcSpecular()
	{
		AsyncGlobal_Rec global(this);

		std::vector<AsyncQuery_Rec> queries(scanner->getContainerPcRAW()->Size());
		for (std::size_t i = 0; i < queries.size(); ++i)
			queries[i].index = i;

		AsyncProcess<AsyncGlobal_Rec, AsyncQuery_Rec, AsyncData_Rec>(
			global, queries,
			AStep_RecPcAtt, BStep_RecPcSpecular, CStep_RecPcAtt,
			asyncSize);

		// NAN fill
		{
			PTR(PcIndex) validFilter(new PcIndex);
			PTR(PcIndex) inValidFilter(new PcIndex);
			validFilter->reserve(pcMED->size());
			inValidFilter->reserve(pcMED->size());
			for (int px = 0; px < pcMED->size(); px++)
			{
				if (specularEstimator->InputPointValid((*pcMED)[px]) && specularEstimator->OutputPointValid((*pcMED)[px]))
					validFilter->push_back(px);
				else
					inValidFilter->push_back(px);
			}

			if ((validFilter->size() > 0) && (inValidFilter->size() > 0))
			{
				PTR(AccMED) validAcc(new KDTreeMED);
				validAcc->setInputCloud(pcMED, validFilter);

				PcMED temp;

				fieldInterpolator->Process(validAcc, pcMED, inValidFilter, temp);

				for (std::size_t idx = 0; idx < inValidFilter->size(); ++idx)
				{
					PointMED& tarP = (*pcMED)[(*inValidFilter)[idx]];
					PointMED& srcP = temp[idx];

					tarP.diffuseAlbedo = srcP.diffuseAlbedo;
					tarP.specularAlbedo = srcP.specularAlbedo;
					tarP.specularSharpness = srcP.specularSharpness;
				}
			}
		}
	}

	void ReconstructorPcOC::ImplementRecPcRefineSpecular()
	{
		AsyncGlobal_Rec global(this);

		std::vector<AsyncQuery_Rec> queries(scanner->getContainerPcRAW()->Size());
		for (std::size_t i = 0; i < queries.size(); ++i)
			queries[i].index = i;

		AsyncProcess<AsyncGlobal_Rec, AsyncQuery_Rec, AsyncData_Rec>(
			global, queries,
			AStep_RecPcAtt, BStep_RecPcRefineSpecular, CStep_RecPcAtt,
			asyncSize);

		// NAN fill
		{
			PTR(PcIndex) validFilter(new PcIndex);
			PTR(PcIndex) inValidFilter(new PcIndex);
			validFilter->reserve(pcMED->size());
			inValidFilter->reserve(pcMED->size());
			for (int px = 0; px < pcMED->size(); px++)
			{
				if (refineSpecularEstimator->InputPointValid((*pcMED)[px]) && refineSpecularEstimator->OutputPointValid((*pcMED)[px]))
					validFilter->push_back(px);
				else
					inValidFilter->push_back(px);
			}

			if ((validFilter->size() > 0) && (inValidFilter->size() > 0))
			{
				PTR(AccMED) validAcc(new KDTreeMED);
				validAcc->setInputCloud(pcMED, validFilter);

				PcMED temp;

				fieldInterpolator->Process(validAcc, pcMED, inValidFilter, temp);

				for (std::size_t idx = 0; idx < inValidFilter->size(); ++idx)
				{
					PointMED& tarP = (*pcMED)[(*inValidFilter)[idx]];
					PointMED& srcP = temp[idx];

					tarP.diffuseAlbedo = srcP.diffuseAlbedo;
					tarP.specularAlbedo = srcP.specularAlbedo;
					tarP.specularSharpness = srcP.specularSharpness;
				}
			}
		}
	}

	void ReconstructorPcOC::ImplementRecSegNDF()
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
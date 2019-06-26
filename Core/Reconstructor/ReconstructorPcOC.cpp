#include <algorithm>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_box.h>

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
		PTR(KdTreeMED) pcRawAcc;
		PTR(KdTreeMED) pcRecAcc;
		PTR(PcIndex) pcReturnIdx;

		AsyncData_Rec() :
			pcRaw(new PcMED),
			pcRec(new PcMED),
			pcRawIdx(new PcIndex),
			pcRecIdx(new PcIndex),
			pcRawAcc(new KdTreeMED),
			pcRecAcc(new KdTreeMED),
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

				global.ptrReconstructorPcOC()->getDownSampler()->Process(data.pcRaw, *data.pcRec);

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
				std::sort(data.pcRecIdx->begin(), data.pcRecIdx->end());

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
			global.ptrReconstructorPcOC()->getOutlierRemover()->Process(data.pcRec, orIndices);
			std::sort(orIndices.begin(), orIndices.end());

			// Combine
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
#ifdef POINT_MED_WITH_NORMAL
		// 
		if (global.ptrReconstructorPcOC()->getNormalEstimator())
		{
			PRINT_INFO("Estimat Normal - Start");

			global.ptrReconstructorPcOC()->getNormalEstimator()->Process(
				data.pcRecAcc, data.pcRec,
				//data.pcRawAcc, data.pcRaw,
				data.pcRec, data.pcRecIdx);

			PRINT_INFO("Estimat Normal - End");
		}
		else
		{
			PRINT_WARNING("normalEstimator is not set, ignore it");
		}
#endif
		return 0;
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

		if (global.ptrReconstructorPcOC()->getUpSampler())
		{
			PRINT_INFO("Upsampling Attribute - Start");

			PcIndex upIdx;
			global.ptrReconstructorPcOC()->getUpSampler()->Process(data.pcRecAcc, data.pcRec, data.pcRaw, upIdx);

			for (std::size_t px = 0; px < upIdx.size(); ++px)
			{
				PointMED& tarP = (*data.pcRaw)[px];

				if (upIdx[px] > 0)
				{
					PointMED& srcP = (*data.pcRec)[upIdx[px]];

#ifdef POINT_MED_WITH_NORMAL
					tarP.normal_x = srcP.normal_x;
					tarP.normal_y = srcP.normal_y;
					tarP.normal_z = srcP.normal_z;
					tarP.curvature = srcP.curvature;
#endif

#ifdef POINT_MED_WITH_SEGLABEL
					tarP.segLabel = srcP.segLabel;
#endif
				}
				else
				{
#ifdef POINT_MED_WITH_NORMAL
					tarP.normal_x = 0.0f;
					tarP.normal_y = 0.0f;
					tarP.normal_z = 0.0f;
					tarP.curvature = 0.0f;
#endif

#ifdef POINT_MED_WITH_SEGLABEL
					tarP.hasSegLabel = -1;
#endif
				}
			}

			PRINT_INFO("Upsampling Attribute - End");
		}
		else
		{
			THROW_EXCEPTION("upSampler is not set");
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

	// Async Reconstruct Attribute - Albedo
	int BStep_RecPcAlbedo(const AsyncGlobal_Rec& global, const AsyncQuery_Rec& query, AsyncData_Rec& data)
	{
#ifdef POINT_MED_WITH_NORMAL
#ifdef POINT_MED_WITH_LABEL
#ifdef POINT_MED_WITH_INTENSITY
		// 
		if (global.ptrReconstructorPcOC()->getAlbedoEstimator())
		{
			PRINT_INFO("Estimat Albedo - Start");

			global.ptrReconstructorPcOC()->getAlbedoEstimator()->Process(
				data.pcRawAcc, data.pcRaw,
				data.pcRec, data.pcRecIdx);

			PRINT_INFO("Estimat Albedo - End");
		}
		else
		{
			PRINT_WARNING("albedoEstimater is not set, ignore it");
		}
#endif
#endif
#endif
		return 0;
	}

	void ReconstructorPcOC::RecPcAlbedo()
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

	void ReconstructorPcOC::RecPcSegment()
	{
	}

	void ReconstructorPcOC::RecSegNDF()
	{
	}

	void ReconstructorPcOC::RecMesh()
	{

	}
}


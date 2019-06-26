#include <algorithm>

#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>

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
		int qi;

		AsyncQuery_Rec(int qi = -1) : qi(qi)
		{}

		virtual int Check(const AsyncGlobal_Rec& global) const
		{
			if (qi < 0) return 1;
			if (qi >= global.ptrReconstructorPcOC()->getScanner()->getContainerPcRAW()->Size()) return 2;
			return 0;
		}

		virtual std::string Info(const AsyncGlobal_Rec& global) const
		{
			std::stringstream strQuery;
			ContainerPcRAW::QuaryMeta quaryMeta = global.ptrReconstructorPcOC()->getScanner()->getContainerPcRAW()->TestQuary(qi);
			strQuery << qi << ": " << quaryMeta.minAABB << ", " << quaryMeta.maxAABB;
			return strQuery.str();
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
			pcRawAcc->setInputCloud(pcRaw);
			pcRecAcc->setInputCloud(pcRec);
		}
	};

	// Async Reconstruct PointCloud
	int AStep_RecPointCloud(const AsyncGlobal_Rec& global, const AsyncQuery_Rec& query, AsyncData_Rec& data)
	{
		// 
		ContainerPcRAW::QuaryData quaryData;
		{
			PRINT_INFO("Quary pointCloud from container - Start");

			quaryData = global.ptrReconstructorPcOC()->getScanner()->getContainerPcRAW()->Quary(query.qi);
			data.pcRaw = quaryData.data;
			data.pcRawIdx = quaryData.index;

			std::stringstream ss;
			ss << "Quary pointCloud from container - End - pcSize: " << data.pcRaw->size() << ", idxSize: " << data.pcRawIdx->size();
			PRINT_INFO(ss.str());
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
				PRINT_INFO("Extract indices - Start");

				pcl::CropBox<PointMED> cb;
				cb.setMin(Eigen::Vector4f(quaryData.minAABB.x(), quaryData.minAABB.y(), quaryData.minAABB.z(), 1.0));
				cb.setMax(Eigen::Vector4f(quaryData.maxAABB.x(), quaryData.maxAABB.y(), quaryData.maxAABB.z(), 1.0));
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
			ss << "Outlier Removal - End - pcSize: " << data.pcRec->size() << ", idxSize: " << data.pcRecIdx->size();
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
				//data.pcRecAcc, data.pcRec,
				data.pcRawAcc, data.pcRaw,
				data.pcRec, data.pcRecIdx,
				*data.pcRec);

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
		PRINT_INFO("Merge - Start");

		pcl::ExtractIndices<PointMED> extract;
		PcMED temp;
		extract.setInputCloud(data.pcRec);
		extract.setIndices(data.pcRecIdx);
		extract.setNegative(false);
		extract.filter(temp);
		(*global.ptrReconstructorPcOC()->getPcMED()) += temp;

		PRINT_INFO("Merge - End - pcSize: " + std::to_string(global.ptrReconstructorPcOC()->getPcMED()->size()));

		return 0;
	}

	void ReconstructorPcOC::RecPointCloud(std::size_t asyncSize)
	{
		AsyncGlobal_Rec global(this);

		std::vector<AsyncQuery_Rec> queries(scanner->getContainerPcRAW()->Size());
		for (std::size_t i = 0; i < queries.size(); ++i)
			queries[i].qi = i;

		AsyncProcess<AsyncGlobal_Rec, AsyncQuery_Rec, AsyncData_Rec>(
			global, queries,
			AStep_RecPointCloud, BStep_RecPointCloud, CStep_RecPointCloud,
			asyncSize);
	}

	// Async Reconstruct Attribute
	int AStep_RecPcAtt(const AsyncGlobal_Rec& global, const AsyncQuery_Rec& query, AsyncData_Rec& data)
	{
		// 
		ContainerPcRAW::QuaryData quaryData;
		{
			PRINT_INFO("Quary pointCloud from container - Start");

			quaryData = global.ptrReconstructorPcOC()->getScanner()->getContainerPcRAW()->Quary(query.qi);
			data.pcRaw = quaryData.data;
			data.pcRawIdx = quaryData.index;

			std::stringstream ss;
			ss << "Quary pointCloud from container - End - pcSize: " << data.pcRaw->size() << ", idxSize: " << data.pcRawIdx->size();
			PRINT_INFO(ss.str());
		}

		// 
		{
			PRINT_INFO("Extract return indices - Start");

			pcl::CropBox<PointMED> cb;
			cb.setMin(Eigen::Vector4f(quaryData.minAABB.x(), quaryData.minAABB.y(), quaryData.minAABB.z(), 1.0));
			cb.setMax(Eigen::Vector4f(quaryData.maxAABB.x(), quaryData.maxAABB.y(), quaryData.maxAABB.z(), 1.0));
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
			cb.setMin(Eigen::Vector4f(quaryData.extMinAABB.x(), quaryData.extMinAABB.y(), quaryData.extMinAABB.z(), 1.0));
			cb.setMax(Eigen::Vector4f(quaryData.extMaxAABB.x(), quaryData.extMaxAABB.y(), quaryData.extMaxAABB.z(), 1.0));
			cb.setInputCloud(global.ptrReconstructorPcOC()->getPcMED());
			cb.filter(*data.pcRecIdx);

			pcl::ExtractIndices<PointMED> extract;
			extract.setInputCloud(global.ptrReconstructorPcOC()->getPcMED());
			extract.setIndices(data.pcRecIdx);
			extract.setNegative(false);
			extract.filter(*data.pcRec);

			//
			data.pcRecIdx->clear();

			std::stringstream ss;
			ss << "Extract pointCloud from pcMED - End - inSize: " << global.ptrReconstructorPcOC()->getPcMED()->size() << ", outSize:" << data.pcRec->size();
			PRINT_INFO(ss.str());

		}

		{
			PRINT_INFO("Extract indices - Start");

			pcl::CropBox<PointMED> cb;
			cb.setMin(Eigen::Vector4f(quaryData.minAABB.x(), quaryData.minAABB.y(), quaryData.minAABB.z(), 1.0));
			cb.setMax(Eigen::Vector4f(quaryData.maxAABB.x(), quaryData.maxAABB.y(), quaryData.maxAABB.z(), 1.0));
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
				data.pcRec, data.pcRecIdx,
				*data.pcRec);

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

	void ReconstructorPcOC::RecPcAlbedo(std::size_t asyncSize)
	{
		AsyncGlobal_Rec global(this);

		std::vector<AsyncQuery_Rec> queries(scanner->getContainerPcRAW()->Size());
		for (std::size_t i = 0; i < queries.size(); ++i)
			queries[i].qi = i;

		AsyncProcess<AsyncGlobal_Rec, AsyncQuery_Rec, AsyncData_Rec>(
			global, queries,
			AStep_RecPcAtt, BStep_RecPcAlbedo, CStep_RecPcAtt,
			asyncSize);
	}

	void ReconstructorPcOC::RecPcSegment(std::size_t asyncSize)
	{
	}

	void ReconstructorPcOC::RecSegNDF(std::size_t asyncSize)
	{
	}

	void ReconstructorPcOC::RecMesh(std::size_t asyncSize)
	{

	}
}


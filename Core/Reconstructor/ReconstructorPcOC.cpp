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
		ReconstructorPcOC* reconstructorPcOC;

		AsyncGlobal_Rec(ReconstructorPcOC* reconstructorPcOC = nullptr)
			: reconstructorPcOC(reconstructorPcOC) {}

		virtual int Check() const
		{
			if (reconstructorPcOC == nullptr) return 1;
			if (!reconstructorPcOC->getScanner()) return 2;
			if (!reconstructorPcOC->getContainerPcRAW()) return 3;
			if (!reconstructorPcOC->getContainerPcNDF()) return 4;
			if (!reconstructorPcOC->getPcMED()) return 5;
			return 0;
		}
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
			if (qi >= global.reconstructorPcOC->getContainerPcRAW()->Size()) return 2;
			return 0;
		}

		virtual std::string Info(const AsyncGlobal_Rec& global) const
		{
			std::stringstream strQuery;
			ContainerPcRAW::QuaryMeta quaryMeta = global.reconstructorPcOC->getContainerPcRAW()->TestQuary(qi);
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

		AsyncData_Rec() :
			pcRaw(new PcMED),
			pcRec(new PcMED),
			pcRawIdx(new PcIndex),
			pcRecIdx(new PcIndex),
			pcRawAcc(new KdTreeMED),
			pcRecAcc(new KdTreeMED)
		{
			pcRawAcc->setInputCloud(pcRaw);
			pcRecAcc->setInputCloud(pcRec);
		}
	};

	// Async Reconstruct PointCloud
	int AStep_RecPointCloud(AsyncGlobal_Rec& global, AsyncQuery_Rec& query, AsyncData_Rec& data)
	{
		// 
		ContainerPcRAW::QuaryData quaryData;
		{
			PRINT_INFO("Quary pointCloud from container - Start");

			quaryData = global.reconstructorPcOC->getContainerPcRAW()->Quary(query.qi);
			data.pcRaw = quaryData.data;
			data.pcRawIdx = quaryData.index;

			std::stringstream ss;
			ss << "Quary pointCloud from container - End - pcSize: " << data.pcRaw->size() << ", idxSize: " << data.pcRawIdx->size();
			PRINT_INFO(ss.str());
		}

		// 
		if (global.reconstructorPcOC->getDownSampler())
		{
			{
				PRINT_INFO("DownSampling - Start");

				global.reconstructorPcOC->getDownSampler()->Process(data.pcRaw, *data.pcRec);

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
			data.pcRec = data.pcRaw;
			data.pcRecIdx = data.pcRawIdx;
			data.pcRecAcc = data.pcRawAcc;
		}

		//
		if (global.reconstructorPcOC->getOutlierRemover())
		{
			PRINT_INFO("Outlier Removal - Start");


			PcIndex orIndices;
			global.reconstructorPcOC->getOutlierRemover()->Process(data.pcRec, orIndices);
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

		return 0;
	}

	int BStep_RecPointCloud(AsyncGlobal_Rec& global, AsyncQuery_Rec& query, AsyncData_Rec& data)
	{
#ifdef POINT_MED_WITH_NORMAL
		// Estimat Normal
		if (global.reconstructorPcOC->getNormalEstimator())
		{
			PRINT_INFO("Estimat Normal - Start");

			global.reconstructorPcOC->getNormalEstimator()->Process(
				data.pcRecAcc, data.pcRec, 
				data.pcRec, data.pcRecIdx,
				*data.pcRec);

			PRINT_INFO("Estimat Normal - End");
		}
#endif
		return 0;
	}

	int CStep_RecPointCloud(AsyncGlobal_Rec& global, AsyncQuery_Rec& query, AsyncData_Rec& data)
	{
		PRINT_INFO("Merge - Start");

		pcl::ExtractIndices<PointMED> extract;
		PcMED temp;
		extract.setInputCloud(data.pcRec);
		extract.setIndices(data.pcRecIdx);
		extract.setNegative(false);
		extract.filter(temp);
		(*global.reconstructorPcOC->getPcMED()) += temp;

		PRINT_INFO("Merge - End - pcSize: " + std::to_string(global.reconstructorPcOC->getPcMED()->size()));

		return 0;
	}

	void ReconstructorPcOC::RecPointCloud()
	{

	}

	void ReconstructorPcOC::RecPcAlbedo()
	{
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


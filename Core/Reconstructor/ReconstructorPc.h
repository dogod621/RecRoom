#pragma once

#include "Common/Common.h"
#include "Container/ContainerPcRAW.h"
#include "Container/ContainerPcNDF.h"
#include "Scanner/ScannerPc.h"

#include "Estimator/EstimatorPc.h"
#include "Filter/FilterPc.h"
#include "Interpolator/InterpolatorPc.h"
#include "Mesher/MesherPc.h"
#include "Sampler/SamplerPc.h"
#include "Segmenter/SegmenterPc.h"

namespace RecRoom
{
	enum ReconstructStatus : Flag
	{
		ReconstructStatus_UNKNOWN = 0,
		POINT_CLOUD = 1 << 1,
		PC_MATERIAL = 1 << 2,
		PC_SEGMENT = 1 << 3,
		SEG_MATERIAL = 1 << 4,
		MESH = 1 << 5
	};

	class ReconstructorPc : public DumpAble
	{
	public:
		using Estimator = EstimatorPc<PointMED, PointMED>;
		using Filter = FilterPc<PointMED>;
		using Interpolator = InterpolatorPc<PointMED, PointMED>;
		using Mesher = MesherPc<PointREC>;
		using Sampler = SamplerPc<PointMED>;
		using Segmenter = SegmenterPc<PointMED>;

	public:
		ReconstructorPc(
			boost::filesystem::path filePath,
			const CONST_PTR(ScannerPc)& scanner,
			const PTR(ContainerPcNDF)& containerPcNDF,
			float res = 0.01f);

	public:
		void DoRecPointCloud();
		void DoRecPcMaterial();
		void DoRecPcSegment();
		void DoRecSegMaterial();
		void DoRecMesh();
		void VisualSegNDFs();
		void VisualRecAtts();

	protected:
		virtual void RecPointCloud() = 0;
		virtual void RecPcMaterial_NDF() = 0;
		virtual void RecPcMaterial_ALBEDO() = 0;
		virtual void RecPcSegment();
		virtual void RecSegMaterial() = 0;
		virtual void RecMesh();

	public:
		float getRes() const { return res; }
		ReconstructStatus getStatus() const { return status; }
		PTR(PcMED) getPcMED() const { return pcMED; }

		CONST_PTR(ScannerPc) getScanner() const { return scanner; }
		PTR(ContainerPcNDF) getContainerPcNDF() const { return containerPcNDF; }

		CONST_PTR(Sampler) getDownSampler() const { return downSampler; }
		CONST_PTR(Interpolator) getInterpolator() const { return interpolator; }
		CONST_PTR(Filter) getOutlierRemover() const { return outlierRemover; }
		CONST_PTR(Estimator) getNormalEstimator() const { return normalEstimator; }
		CONST_PTR(Estimator) getNDFEstimator() const { return ndfEstimator; }
		CONST_PTR(Estimator) getAlbedoEstimator() const { return albedoEstimator; }
		CONST_PTR(Segmenter) getSegmenter() const { return segmenter; }
		CONST_PTR(Mesher) getMesher() const { return mesher; }

		void setDownSampler(const CONST_PTR(Sampler)& v) { downSampler = v; }
		void setInterpolator(const CONST_PTR(Interpolator)& v) 
		{ 
			if (!v)
			{
				THROW_EXCEPTION("interpolator is not set");
			}
			else
			{
				interpolator = v;
			}
		}
		void setOutlierRemover(const CONST_PTR(Filter)& v) { outlierRemover = v; }
		void setNormalEstimator(const CONST_PTR(Estimator)& v) { normalEstimator = v; }
		void setNDFEstimator(const CONST_PTR(Estimator)& v) { ndfEstimator = v; }
		void setAlbedoEstimator(const CONST_PTR(Estimator)& v) { albedoEstimator = v; }
		void setSegmenter(const CONST_PTR(Segmenter)& v) { segmenter = v; }
		void setMesher(const CONST_PTR(Mesher)& v) { mesher = v;}
		
	protected:
		float res;
		ReconstructStatus status;
		PTR(PcMED) pcMED;
		PTR(Mesh) mesh;

		CONST_PTR(ScannerPc) scanner; 
		PTR(ContainerPcNDF) containerPcNDF;

		CONST_PTR(Sampler) downSampler;
		CONST_PTR(Interpolator) interpolator;
		CONST_PTR(Filter) outlierRemover;
		CONST_PTR(Estimator) normalEstimator;
		CONST_PTR(Estimator) ndfEstimator;
		CONST_PTR(Estimator) albedoEstimator;
		CONST_PTR(Segmenter) segmenter;
		CONST_PTR(Mesher) mesher;

		virtual void Load();
		virtual void Dump() const;
		virtual void Load(const nlohmann::json& j);
		virtual void Dump(nlohmann::json& j) const;
		virtual bool CheckExist() const;

		void PcMEDRemoveNonFinite();

		void PcMEDRemoveDuplicate();
	};
}

#include "ReconstructorPc.hpp"
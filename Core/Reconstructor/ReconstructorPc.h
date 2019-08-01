#pragma once

#include "Common/Common.h"
#include "Container/ContainerPcRAW.h"
#include "Container/ContainerPcNDF.h"
#include "Scanner/ScannerPc.h"

#include "Estimator/EstimatorPc.h"
#include "Filter/FilterPc.h"
#include "Interpolator/InterpolatorPcNearest.h"
#include "Mesher/MesherPc.h"
#include "Sampler/SamplerPc.h"
#include "Segmenter/SegmenterPc.h"

namespace RecRoom
{
	enum ReconstructStatus : Flag
	{
		ReconstructStatus_UNKNOWN = 0,
		POINT_CLOUD = 1 << 1,
		PC_NORMAL = 1 << 2,
		PC_ALBEDO = 1 << 3,
		PC_SHARPNESS = 1 << 4,
		PC_SEGMENT = 1 << 5,
		SEG_MATERIAL = 1 << 6,
		MESH = 1 << 7,
		REMESH = 1 << 8
	};

	class ReconstructorPc : public DumpAble
	{
	public:
		using Estimator = EstimatorPc<PointMED, PointMED>;
		using Filter = FilterPc<PointMED>;
		using InterpolatorMED = InterpolatorPc<PointMED, PointMED>;
		using InterpolatorREC = InterpolatorPc<PointREC, PointREC>;
		using Mesher = MesherPc<PointREC>;
		using MesherPreFilter = FilterPc<PointREC>;
		using MesherPreSampler = SamplerPc<PointREC>;
		using Sampler = SamplerPc<PointMED>;
		using Segmenter = SegmenterPc<PointMED>;
		
	public:
		ReconstructorPc(
			boost::filesystem::path filePath,
			const CONST_PTR(ScannerPc)& scanner,
			const PTR(ContainerPcNDF)& containerPcNDF,
			const CONST_PTR(InterpolatorMED)& fieldInterpolatorMED = CONST_PTR(InterpolatorMED)(new InterpolatorPcNearest<PointMED, PointMED>),
			const CONST_PTR(InterpolatorREC)& fieldInterpolatorREC = CONST_PTR(InterpolatorREC)(new InterpolatorPcNearest<PointREC, PointREC>),
			bool useVNN = true,
			float resVNN = 0.01f);

	public:
		void RecPointCloud();
		void RecPcNormal();
		void RecPcAlbedo();
		void RecPcSharpness();
		void RecPcSegment();
		void RecSegMaterial();
		void RecMesh();
		void RecReMesh(float holeSize);
		void VisualSegNDFs();
		void VisualRecAtts();

	protected:
		virtual void ImplementRecPointCloud() = 0;
		virtual void ImplementRecPcNormal() = 0;
		virtual void ImplementRecPcAlbedo() = 0;
		virtual void ImplementRecPcSharpness() = 0;
		virtual void ImplementRecPcSegment();
		virtual void ImplementRecSegMaterial() = 0;
		virtual void ImplementRecMesh();
		virtual void ImplementRecReMesh(float holeSize);

	public:
		bool getUseVNN() const { return useVNN; }
		float getResVNN() const { return resVNN; }
		ReconstructStatus getStatus() const { return status; }
		PTR(PcMED) getPcMED() const { return pcMED; }

		CONST_PTR(ScannerPc) getScanner() const { return scanner; }
		PTR(ContainerPcNDF) getContainerPcNDF() const { return containerPcNDF; }

		CONST_PTR(Sampler) getDownSampler() const { return downSampler; }
		CONST_PTR(InterpolatorMED) getFieldInterpolatorMED() const { return fieldInterpolatorMED; }
		CONST_PTR(InterpolatorREC) getFieldInterpolatorREC() const { return fieldInterpolatorREC; }
		CONST_PTR(Filter) getOutlierRemover() const { return outlierRemover; }
		CONST_PTR(Estimator) getNormalEstimator() const { return normalEstimator; }
		CONST_PTR(Estimator) getAlbedoEstimator() const { return albedoEstimator; }
		CONST_PTR(Estimator) getSharpnessEstimator() const { return sharpnessEstimator; }
		CONST_PTR(Segmenter) getSegmenter() const { return segmenter; }
		CONST_PTR(Mesher) getMesher() const { return mesher; }

		void setDownSampler(const CONST_PTR(Sampler)& v) 
		{ 
			downSampler = v; 
			PTR(Sampler) downSamplerTemp = boost::const_pointer_cast<Sampler>(downSampler);
			if (downSamplerTemp)
				downSamplerTemp->setFieldInterpolator(fieldInterpolatorMED);
		}
		void setFieldInterpolatorMED(const CONST_PTR(InterpolatorMED)& v)
		{ 
			if (!v)
			{
				THROW_EXCEPTION("fieldInterpolatorMED is not set");
			}
			else
			{
				fieldInterpolatorMED = v;

				PTR(Sampler) downSamplerTemp = boost::const_pointer_cast<Sampler>(downSampler);
				if (downSamplerTemp)
					downSamplerTemp->setFieldInterpolator(fieldInterpolatorMED);
			}
		}
		void setFieldInterpolatorREC(const CONST_PTR(InterpolatorREC)& v)
		{
			if (!v)
			{
				THROW_EXCEPTION("fieldInterpolatorREC is not set");
			}
			else
			{
				fieldInterpolatorREC = v;
				PTR(Mesher) mesherTemp = boost::const_pointer_cast<Mesher>(mesher);
				PTR(MesherPreSampler) mesherPreSamplerTemp = boost::const_pointer_cast<MesherPreSampler>(mesherPreSampler);

				if (mesherTemp)
					mesherTemp->setFieldInterpolator(fieldInterpolatorREC);
				if (mesherPreSamplerTemp)
					mesherPreSamplerTemp->setFieldInterpolator(fieldInterpolatorREC);
			}
		}
		void setOutlierRemover(const CONST_PTR(Filter)& v) { outlierRemover = v; }
		void setNormalEstimator(const CONST_PTR(Estimator)& v) { normalEstimator = v; }
		void setAlbedoEstimator(const CONST_PTR(Estimator)& v) { albedoEstimator = v; }
		void setSharpnessEstimator(const CONST_PTR(Estimator)& v) { sharpnessEstimator = v; }
		void setSegmenter(const CONST_PTR(Segmenter)& v) { segmenter = v; }
		void setMesher(const CONST_PTR(Mesher)& v) 
		{ 
			mesher = v;
			PTR(Mesher) mesherTemp = boost::const_pointer_cast<Mesher>(mesher);
			if (mesherTemp)
				mesherTemp->setFieldInterpolator(fieldInterpolatorREC);
		}
		void setMeshPreFilter(const CONST_PTR(MesherPreFilter)& v) { mesherPreFilter = v; }
		void setMesherPreSampler(const CONST_PTR(MesherPreSampler)& v) 
		{
			mesherPreSampler = v; 
			PTR(MesherPreSampler) mesherPreSamplerTemp = boost::const_pointer_cast<MesherPreSampler>(mesherPreSampler);
			if (mesherPreSamplerTemp)
				mesherPreSamplerTemp->setFieldInterpolator(fieldInterpolatorREC);
		}

	protected:
		bool useVNN;
		float resVNN;
		ReconstructStatus status;
		PTR(PcMED) pcMED;
		PTR(Mesh) mesh;

		CONST_PTR(ScannerPc) scanner; 
		PTR(ContainerPcNDF) containerPcNDF;

		CONST_PTR(Sampler) downSampler;
		CONST_PTR(InterpolatorMED) fieldInterpolatorMED;
		CONST_PTR(InterpolatorREC) fieldInterpolatorREC;
		CONST_PTR(Filter) outlierRemover;
		CONST_PTR(Estimator) normalEstimator;
		CONST_PTR(Estimator) albedoEstimator;
		CONST_PTR(Estimator) sharpnessEstimator;
		CONST_PTR(Segmenter) segmenter;
		CONST_PTR(Sampler) preprocessSampler;
		CONST_PTR(Filter) preprocessFilter;
		CONST_PTR(Mesher) mesher;
		CONST_PTR(MesherPreFilter) mesherPreFilter;
		CONST_PTR(MesherPreSampler) mesherPreSampler;

		virtual void Load();
		virtual void Dump() const;
		virtual void Load(const nlohmann::json& j);
		virtual void Dump(nlohmann::json& j) const;
		virtual bool CheckExist() const;
	};
}

#include "ReconstructorPc.hpp"
#pragma once

#include "Common/Common.h"
#include "Container/ContainerPcRAW.h"
#include "Container/ContainerPcNDF.h"
#include "Scanner/ScannerPc.h"

#include "Estimator/EstimatorPc.h"
#include "Estimator/EstimatorPcNDF.h"
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
		PC_DIFFUSE = 1 << 3,
		PC_SPECULAR = 1 << 4,
		PC_REFINE_SPECULAR = 1 << 5,
		PC_SEGMENT = 1 << 6,
		SEG_NDF = 1 << 7,
		SEG_MATERIAL = 1 << 8,
		MESH_PREPROCESS = 1 << 9,
		MESH = 1 << 10,
		MESH_POSTPROCESS = 1 << 11
	};

	class ReconstructorPc : public DumpAble
	{
	public:
		using Estimator = EstimatorPc<PointMED, PointMED>;
		using EstimatorNDF = EstimatorPcNDF<PointMED, PointMED>;
		using Filter = FilterPc<PointMED>;
		using Interpolator = InterpolatorPc<PointMED, PointMED>;
		using Sampler = SamplerPc<PointMED>;
		using Segmenter = SegmenterPc<PointMED>;

		using Mesher = MesherPc<PointREC>;
		using MeshFilter = FilterPc<PointREC>;
		using MeshInterpolator = InterpolatorPc<PointREC, PointREC>;
		using MeshSampler = SamplerPc<PointREC>;
		
	public:
		ReconstructorPc(
			boost::filesystem::path filePath,
			const CONST_PTR(ScannerPc)& scanner,
			const PTR(ContainerPcNDF)& containerPcNDF,
			const CONST_PTR(Interpolator)& fieldInterpolator = CONST_PTR(Interpolator)(new InterpolatorPcNearest<PointMED, PointMED>),
			const CONST_PTR(MeshInterpolator)& meshFieldInterpolator = CONST_PTR(MeshInterpolator)(new InterpolatorPcNearest<PointREC, PointREC>),
			bool useVNN = true,
			float resVNN = 0.01f);

	public:
		void RecPointCloud();
		void RecPcNormal();
		void RecPcDiffuse();
		void RecPcSpecular();
		void RecPcRefineSpecular();
		void RecPcSegment();
		void RecSegNDF();
		void RecSegMaterial();
		void RecMeshPreprocess();
		void RecMesh();
		void RecMeshPostprocess(float holeSize = -1.0,
			int laplacianNumIter = 20, float laplacianRelaxationFactor = 0.01f, float laplacianConvergence = 0.0,
			bool laplacianSmoothFeatureEdge = false, float laplacianFeatureAngle = 45.f,
			bool laplacianSmoothBoundary = true, float laplacianEdgeAngle = 15.f);
		void VisualSegNDFs();
		void VisualRecAtts();

	protected:
		virtual void ImplementRecPointCloud() = 0;
		virtual void ImplementRecPcNormal() = 0;
		virtual void ImplementRecPcDiffuse() = 0;
		virtual void ImplementRecPcSpecular() = 0;
		virtual void ImplementRecPcRefineSpecular() = 0;
		virtual void ImplementRecPcSegment();
		virtual void ImplementRecSegNDF() = 0;
		virtual void ImplementRecSegMaterial();
		virtual void ImplementRecMeshPreprocess();
		virtual void ImplementRecMesh();
		virtual void ImplementRecMeshPostprocess(float holeSize,
			int laplacianNumIter, float laplacianRelaxationFactor, float laplacianConvergence,
			bool laplacianSmoothFeatureEdge, float laplacianFeatureAngle,
			bool laplacianSmoothBoundary, float laplacianEdgeAngle);

	public:
		bool getUseVNN() const { return useVNN; }
		float getResVNN() const { return resVNN; }
		ReconstructStatus getStatus() const { return status; }
		PTR(PcMED) getPcMED() const { return pcMED; }
		PTR(PcSoftLabel) getPcSoftLabel() const { return pcSoftLabel; }
		PTR(PcMED) getPcSegMaterial() const { return pcSegMaterial; }
		PTR(PcREC) getPcREC() const { return pcREC; }
		PTR(Mesh) getMesh() const { return mesh; }

		CONST_PTR(ScannerPc) getScanner() const { return scanner; }
		PTR(ContainerPcNDF) getContainerPcNDF() const { return containerPcNDF; }

		CONST_PTR(Sampler) getDownSampler() const { return downSampler; }
		CONST_PTR(Interpolator) getFieldInterpolator() const { return fieldInterpolator; }
		CONST_PTR(Estimator) getNormalEstimator() const { return normalEstimator; }
		CONST_PTR(Estimator) getDiffuseEstimator() const { return diffuseEstimator; }
		CONST_PTR(EstimatorNDF) getSpecularEstimator() const { return specularEstimator; }
		CONST_PTR(Estimator) getRefineSpecularEstimator() const { return refineSpecularEstimator; }
		CONST_PTR(Segmenter) getSegmenter() const { return segmenter; }
		CONST_PTR(Mesher) getMesher() const { return mesher; }
		CONST_PTR(MeshFilter) getMeshOutlierRemover() const { return meshOutlierRemover; }
		CONST_PTR(MeshFilter) getMeshFilter() const { return meshFilter; }
		CONST_PTR(MeshInterpolator) getMeshFieldInterpolator() const { return meshFieldInterpolator; }
		CONST_PTR(MeshSampler) getMeshSampler() const { return meshSampler; }

		void setDownSampler(const CONST_PTR(Sampler)& v) 
		{ 
			downSampler = v; 
			PTR(Sampler) downSamplerTemp = boost::const_pointer_cast<Sampler>(downSampler);
			if (downSamplerTemp)
				downSamplerTemp->setFieldInterpolator(fieldInterpolator);
		}
		void setFieldInterpolator(const CONST_PTR(Interpolator)& v)
		{ 
			if (!v)
			{
				THROW_EXCEPTION("fieldInterpolator is not set");
			}
			else
			{
				fieldInterpolator = v;

				PTR(Sampler) downSamplerTemp = boost::const_pointer_cast<Sampler>(downSampler);
				if (downSamplerTemp)
					downSamplerTemp->setFieldInterpolator(fieldInterpolator);
			}
		}
		void setNormalEstimator(const CONST_PTR(Estimator)& v) { normalEstimator = v; }
		void setDiffuseEstimator(const CONST_PTR(Estimator)& v) { diffuseEstimator = v; }
		void setSpecularEstimator(const CONST_PTR(EstimatorNDF)& v) { specularEstimator = v; }
		void setRefineSpecularEstimator(const CONST_PTR(Estimator)& v) { refineSpecularEstimator = v; }
		void setSegmenter(const CONST_PTR(Segmenter)& v) { segmenter = v; }
		void setMesher(const CONST_PTR(Mesher)& v) 
		{ 
			mesher = v;
			PTR(Mesher) mesherTemp = boost::const_pointer_cast<Mesher>(mesher);
			if (mesherTemp)
				mesherTemp->setFieldInterpolator(meshFieldInterpolator);
		}
		void setMeshOutlierRemover(const CONST_PTR(MeshFilter)& v) { meshOutlierRemover = v; }
		void setMeshFilter(const CONST_PTR(MeshFilter)& v) { meshFilter = v; }
		void setMeshFieldInterpolator(const CONST_PTR(MeshInterpolator)& v)
		{
			if (!v)
			{
				THROW_EXCEPTION("meshFieldInterpolator is not set");
			}
			else
			{
				meshFieldInterpolator = v;
				PTR(Mesher) mesherTemp = boost::const_pointer_cast<Mesher>(mesher);
				PTR(MeshSampler) meshSamplerTemp = boost::const_pointer_cast<MeshSampler>(meshSampler);

				if (mesherTemp)
					mesherTemp->setFieldInterpolator(meshFieldInterpolator);
				if (meshSamplerTemp)
					meshSamplerTemp->setFieldInterpolator(meshFieldInterpolator);
			}
		}
		void setMeshSampler(const CONST_PTR(MeshSampler)& v) 
		{
			meshSampler = v; 
			PTR(MeshSampler) meshSamplerTemp = boost::const_pointer_cast<MeshSampler>(meshSampler);
			if (meshSamplerTemp)
				meshSamplerTemp->setFieldInterpolator(meshFieldInterpolator);
		}

	protected:
		bool useVNN;
		float resVNN;
		ReconstructStatus status;
		PTR(PcMED) pcMED;
		PTR(PcSoftLabel) pcSoftLabel;
		PTR(PcMED) pcSegMaterial;
		PTR(PcREC) pcREC;
		PTR(Mesh) mesh;

		CONST_PTR(ScannerPc) scanner; 
		PTR(ContainerPcNDF) containerPcNDF;

		CONST_PTR(Sampler) downSampler;
		CONST_PTR(Interpolator) fieldInterpolator;
		CONST_PTR(Estimator) normalEstimator;
		CONST_PTR(Estimator) diffuseEstimator;
		CONST_PTR(EstimatorNDF) specularEstimator;
		CONST_PTR(Estimator) refineSpecularEstimator;
		CONST_PTR(Segmenter) segmenter;
		
		CONST_PTR(Mesher) mesher;
		CONST_PTR(MeshFilter) meshOutlierRemover;
		CONST_PTR(MeshFilter) meshFilter;
		CONST_PTR(MeshInterpolator) meshFieldInterpolator;
		CONST_PTR(MeshSampler) meshSampler;

		virtual void Load();
		virtual void Dump() const;
		virtual void Load(const nlohmann::json& j);
		virtual void Dump(nlohmann::json& j) const;
		virtual bool CheckExist() const;
	};
}

#include "ReconstructorPc.hpp"
#pragma once

#include "Common/Common.h"
#include "Container/ContainerPcRAW.h"
#include "Container/ContainerPcNDF.h"
#include "Scanner/ScannerPc.h"

#include "Sampler/SamplerPc.h"
#include "Cropper/CropperPc.h"
#include "Estimator/EstimatorPc.h"
#include "Segmenter/SegmenterPc.h"
#include "Mesher/MesherPc.h"

namespace RecRoom
{
	enum ReconstructStatus : Flag
	{
		ReconstructStatus_UNKNOWN = 0,
		POINT_CLOUD = 1 << 1,
		PC_ALBEDO = 1 << 2,
		PC_SEGMENT = 1 << 3,
		SEG_NDF = 1 << 4,
		MESH = 1 << 5
	};

	class ReconstructorPc
	{
	public:
		ReconstructorPc(
			boost::filesystem::path filePath,
			const CONST_PTR(ScannerPc)& scanner,
			const CONST_PTR(ContainerPcRAW)& containerPcRAW,
			const PTR(ContainerPcNDF)& containerPcNDF);

	public:
		void WarpRecPointCloud();
		void WarpRecPcAlbedo();
		void WarpRecPcSegment();
		void WarpRecSegNDF();
		void WarpRecMesh();

	public:
		//virtual void Process(pcl::PolygonMesh& out) const = 0;
		virtual void RecPointCloud() = 0;
		virtual void RecPcAlbedo() = 0;
		virtual void RecPcSegment() = 0;
		virtual void RecSegNDF() = 0;
		virtual void RecMesh() = 0;
		virtual void SynthScanImages(const boost::filesystem::path& scanImagePath, const Mapping mapping, unsigned int width, const unsigned int height) = 0;

	public:
		boost::filesystem::path getFilePath() const { return filePath; }
		ReconstructStatus getStatus() const { return status; }
		PTR(PcMED) getPcMED() const { return pcMED; }

		CONST_PTR(ScannerPc) getScanner() const { return scanner; }
		CONST_PTR(ContainerPcRAW) getContainerPcRAW() const { return containerPcRAW; }
		PTR(ContainerPcNDF) getContainerPcNDF() const { return containerPcNDF; }

		CONST_PTR(SamplerPc) getDownSampler() const { return downSampler; }
		CONST_PTR(SamplerPc) getUpSampler() const { return upSampler; }
		CONST_PTR(CropperPc) getOutlierRemover() const { return outlierRemover; }
		CONST_PTR(EstimatorPc) getNormalEstimator() const { return normalEstimator; }
		CONST_PTR(EstimatorPc) getAlbedoEstimator() const { return albedoEstimator; }
		CONST_PTR(SegmenterPc) getSegmenter() const { return segmenter; }
		CONST_PTR(MesherPc) getMesher() const { return mesher; }

		void setDownSampler(CONST_PTR(SamplerPc) downSampler_) { downSampler = downSampler_; }
		void setUpSampler(CONST_PTR(SamplerPc) upSampler_) { upSampler = upSampler_; }
		void setOutlierRemover(CONST_PTR(CropperPc) outlierRemover_) { outlierRemover = outlierRemover_; }
		void setNormalEstimator(CONST_PTR(EstimatorPc) normalEstimator_) { normalEstimator = normalEstimator_; }
		void setAlbedoEstimator(CONST_PTR(EstimatorPc) albedoEstimator_) { albedoEstimator = albedoEstimator_; }
		void setSegmenter(CONST_PTR(SegmenterPc) segmenter_) { segmenter = segmenter_; }
		void setMesher(CONST_PTR(MesherPc) mesher_) { mesher = mesher_;}
		
	protected:
		boost::filesystem::path filePath;
		ReconstructStatus status;
		PTR(PcMED) pcMED;

		CONST_PTR(ScannerPc) scanner; 
		CONST_PTR(ContainerPcRAW) containerPcRAW;
		PTR(ContainerPcNDF) containerPcNDF;

		CONST_PTR(SamplerPc) downSampler;
		CONST_PTR(SamplerPc) upSampler;
		CONST_PTR(CropperPc) outlierRemover;
		CONST_PTR(EstimatorPc) normalEstimator;
		CONST_PTR(EstimatorPc) albedoEstimator;
		CONST_PTR(SegmenterPc) segmenter;
		CONST_PTR(MesherPc) mesher;

		void LoadMeta();
		void DumpMeta() const;
	};
}

#include "ReconstructorPc.hpp"
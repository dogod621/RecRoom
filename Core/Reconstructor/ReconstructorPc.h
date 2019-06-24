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
	class ReconstructorPc
	{
	public:
		ReconstructorPc(
			boost::filesystem::path filePath,
			const CONST_PTR(ScannerPc)& scanner,
			const CONST_PTR(ContainerPcRAW)& containerPcRAW,
			const PTR(ContainerPcNDF)& containerPcNDF);

	public:
		virtual void Process(pcl::PolygonMesh& out) const = 0;

	public:
		CONST_PTR(ScannerPc) getScanner() const { return scanner; }
		CONST_PTR(ContainerPcRAW) getContainerPcRAW() const { return containerPcRAW; }
		PTR(ContainerPcNDF) getContainerPcNDF() const { return containerPcNDF; }

		void setDownSampler(CONST_PTR(SamplerPc) downSampler_) { downSampler = downSampler_; }
		void setUpSampler(CONST_PTR(SamplerPc) upSampler_) { upSampler = upSampler_; }
		void setOutlierRemover(CONST_PTR(CropperPc) outlierRemover_) { outlierRemover = outlierRemover_; }
		void setNormalEstimator(CONST_PTR(EstimatorPc) normalEstimator_) { normalEstimator = normalEstimator_; }
		void setAlbedoEstimator(CONST_PTR(EstimatorPc) albedoEstimator_) { albedoEstimator = albedoEstimator_; }
		void setSegmenter(CONST_PTR(SegmenterPc) segmenter_) { segmenter = segmenter_; }
		void setMesher(CONST_PTR(MesherPc) mesher_) { mesher = mesher_;}

	protected:
		boost::filesystem::path filePath;
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
	};
}

#include "ReconstructorPc.hpp"
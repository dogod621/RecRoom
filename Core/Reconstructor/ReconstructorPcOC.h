#pragma once

#include "ReconstructorPc.h"

namespace RecRoom
{
	class ReconstructorPcOC : public ReconstructorPc, public AsyncAble
	{
	public:
		using Estimator = ReconstructorPc::Estimator;
		using Filter = ReconstructorPc::Filter;
		using InterpolatorMED = ReconstructorPc::InterpolatorMED;
		using InterpolatorREC = ReconstructorPc::InterpolatorREC;
		using Mesher = ReconstructorPc::Mesher;
		using Sampler = ReconstructorPc::Sampler;
		using Segmenter = ReconstructorPc::Segmenter;

	public:
		ReconstructorPcOC(
			boost::filesystem::path filePath,
			const CONST_PTR(ScannerPc)& scanner,
			const PTR(ContainerPcNDF)& containerPcNDF,
			const CONST_PTR(InterpolatorMED)& fieldInterpolatorMED = CONST_PTR(InterpolatorMED)(new InterpolatorPcNearest<PointMED, PointMED>),
			const CONST_PTR(InterpolatorREC)& fieldInterpolatorREC = CONST_PTR(InterpolatorREC)(new InterpolatorPcNearest<PointREC, PointREC>),
			bool useVNN = true,
			float resVNN = 0.01f)
			: ReconstructorPc(filePath, scanner, containerPcNDF, fieldInterpolatorMED, fieldInterpolatorREC, useVNN, resVNN), AsyncAble(1){}
		
	protected:
		virtual void ImplementRecPointCloud();
		virtual void ImplementRecPcNormal();
		virtual void ImplementRecPcAlbedo();
		virtual void ImplementRecPcSharpness();
		virtual void ImplementRecSegMaterial();
	};
}

#include "ReconstructorPcOC.hpp"
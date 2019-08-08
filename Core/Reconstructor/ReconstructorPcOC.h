#pragma once

#include "ReconstructorPc.h"

namespace RecRoom
{
	class ReconstructorPcOC : public ReconstructorPc, public AsyncAble
	{
	public:
		using Estimator = ReconstructorPc::Estimator;
		using Filter = ReconstructorPc::Filter;
		using Interpolator = ReconstructorPc::Interpolator;
		using Sampler = ReconstructorPc::Sampler;
		using Segmenter = ReconstructorPc::Segmenter;

		using Mesher = ReconstructorPc::Mesher;
		using MeshFilter = ReconstructorPc::MeshFilter;
		using MeshInterpolator = ReconstructorPc::MeshInterpolator;
		using MeshSampler = ReconstructorPc::MeshSampler;

	public:
		ReconstructorPcOC(
			boost::filesystem::path filePath,
			const CONST_PTR(ScannerPc)& scanner,
			const PTR(ContainerPcNDF)& containerPcNDF,
			const CONST_PTR(Interpolator)& fieldInterpolator = CONST_PTR(Interpolator)(new InterpolatorPcNearest<PointMED, PointMED>),
			const CONST_PTR(MeshInterpolator)& meshFieldInterpolator = CONST_PTR(MeshInterpolator)(new InterpolatorPcNearest<PointREC, PointREC>),
			bool useVNN = true,
			float resVNN = 0.01f)
			: ReconstructorPc(filePath, scanner, containerPcNDF, fieldInterpolator, meshFieldInterpolator, useVNN, resVNN), AsyncAble(1){}
		
	protected:
		virtual void ImplementRecPointCloud();
		virtual void ImplementRecPcNormal();
		virtual void ImplementRecPcDiffuse();
		virtual void ImplementRecPcSpecular();
		virtual void ImplementRecPcRefineSpecular();
		virtual void ImplementRecSegNDF();
	};
}

#include "ReconstructorPcOC.hpp"
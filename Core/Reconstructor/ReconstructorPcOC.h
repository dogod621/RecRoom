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
		using Mesher = ReconstructorPc::Mesher;
		using Sampler = ReconstructorPc::Sampler;
		using Segmenter = ReconstructorPc::Segmenter;

	public:
		ReconstructorPcOC(
			boost::filesystem::path filePath,
			const CONST_PTR(ScannerPc)& scanner,
			const PTR(ContainerPcNDF)& containerPcNDF,
			bool useVNN = true,
			float resVNN = 0.01f)
			: ReconstructorPc(filePath, scanner, containerPcNDF, useVNN, resVNN), AsyncAble(1){}
		
	protected:
		virtual void ImplementRecPointCloud();
		virtual void ImplementRecPcNormal();
		virtual void ImplementRecPcAlbedo();
		virtual void ImplementRecPcSharpness();
		virtual void ImplementRecSegMaterial();
	};
}

#include "ReconstructorPcOC.hpp"
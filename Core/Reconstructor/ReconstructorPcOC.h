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
			float res = 0.01f)
			: ReconstructorPc(filePath, scanner, containerPcNDF, res), AsyncAble(1), useVNN(true){}

	public:
		bool getUseVNN() const { return useVNN; }

	protected:
		virtual void ImplementRecPointCloud();
		virtual void ImplementRecPcNormal();
		virtual void ImplementRecPcAlbedo();
		virtual void ImplementRecPcSharpness();
		virtual void ImplementRecSegMaterial();

	protected:
		bool useVNN;
	};
}

#include "ReconstructorPcOC.hpp"
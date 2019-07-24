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
			const PTR(ContainerPcNDF)& containerPcNDF)
			: ReconstructorPc(filePath, scanner, containerPcNDF), AsyncAble(1) {}

	protected:
		virtual void RecPointCloud();
		virtual void RecPcMaterial_NDF();
		virtual void RecPcMaterial_ALBEDO();
		virtual void RecSegMaterial();
	};
}

#include "ReconstructorPcOC.hpp"
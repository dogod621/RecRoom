#pragma once

#include "ReconstructorPc.h"

namespace RecRoom
{
	class ReconstructorPcOC : public ReconstructorPc, public AsyncAble
	{
	public:
		ReconstructorPcOC(
			boost::filesystem::path filePath,
			const CONST_PTR(ScannerPc)& scanner,
			const PTR(ContainerPcNDF)& containerPcNDF)
			: ReconstructorPc(filePath, scanner, containerPcNDF), AsyncAble(1) {}

	protected:
		//virtual void Process(pcl::PolygonMesh& out) const = 0;
		virtual void RecPointCloud();
		virtual void RecPcAlbedo();
		virtual void RecSegNDF();
	};
}

#include "ReconstructorPcOC.hpp"
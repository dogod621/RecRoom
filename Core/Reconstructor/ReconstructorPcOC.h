#pragma once

#include "ReconstructorPc.h"

namespace RecRoom
{
	class ReconstructorPcOC : public ReconstructorPc
	{
	public:
		ReconstructorPcOC(
			boost::filesystem::path filePath,
			const CONST_PTR(ScannerPc)& scanner,
			const CONST_PTR(ContainerPcRAW)& containerPcRAW,
			const PTR(ContainerPcNDF)& containerPcNDF)
			: ReconstructorPc(filePath, scanner, containerPcRAW, containerPcNDF) {}

	public:
		//virtual void Process(pcl::PolygonMesh& out) const = 0;
		virtual void RecPointCloud();
		virtual void RecPcAlbedo();
		virtual void RecPcSegment();
		virtual void RecSegNDF();
		virtual void RecMesh();
	};
}

#include "ReconstructorPcOC.hpp"
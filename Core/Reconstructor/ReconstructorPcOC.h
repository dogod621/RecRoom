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
			const PTR(ContainerPcNDF)& containerPcNDF)
			: ReconstructorPc(filePath, scanner, containerPcNDF) {}

	public:
		//virtual void Process(pcl::PolygonMesh& out) const = 0;
		virtual void RecPointCloud() { RecPointCloud(1); }
		virtual void RecPcAlbedo() { RecPcAlbedo(1); }
		virtual void RecPcSegment() { RecPcSegment(1); }
		virtual void RecSegNDF() { RecSegNDF(1); }
		virtual void RecMesh() { RecMesh(1); }

		virtual void RecPointCloud(std::size_t asyncSize);
		virtual void RecPcAlbedo(std::size_t asyncSize);
		virtual void RecPcSegment(std::size_t asyncSize);
		virtual void RecSegNDF(std::size_t asyncSize);
		virtual void RecMesh(std::size_t asyncSize);
	};
}

#include "ReconstructorPcOC.hpp"
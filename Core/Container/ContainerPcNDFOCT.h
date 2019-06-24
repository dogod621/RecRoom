#pragma once

#include <pcl/outofcore/outofcore.h>
#include <pcl/outofcore/outofcore_impl.h>

#include "ContainerPcNDF.h"

namespace RecRoom
{
	class ContainerPcNDFOCT : public ContainerPcNDF
	{
	public:
		using OCTDC = pcl::outofcore::OutofcoreOctreeDiskContainer<PointNDF>;
		using OCT = pcl::outofcore::OutofcoreOctreeBase<OCTDC, PointNDF>;

	public:
		ContainerPcNDFOCT(const boost::filesystem::path& filePath, bool createNew = true);

	public:
		virtual void Merge(const PTR(PcNDF)& v) { oct->addPointCloud(v); }
		virtual std::size_t Size() const { return 0; }
		virtual PTR(PcNDF) Quary(std::size_t i) const;

	protected:
		boost::filesystem::path filePath;
		PTR(OCT) oct;
	};
}

#include "ContainerPcNDFOCT.hpp"
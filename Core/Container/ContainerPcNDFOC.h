#pragma once

#include <pcl/outofcore/outofcore.h>
#include <pcl/outofcore/outofcore_impl.h>

#include "ContainerPcNDF.h"

namespace RecRoom
{
	class ContainerPcNDFOC : public ContainerPcNDF
	{
	public:
		using OCTDC = pcl::outofcore::OutofcoreOctreeDiskContainer<PointNDF>;
		using OCT = pcl::outofcore::OutofcoreOctreeBase<OCTDC, PointNDF>;

	public:
		ContainerPcNDFOC(const boost::filesystem::path& filePath);

	public:
		virtual void Merge(const PTR(PcNDF)& v) { oct->addPointCloud(v); }
		virtual std::size_t Size() const { return std::numeric_limits<unsigned short>::max(); }
		virtual PTR(PcNDF) Quary(std::size_t i) const;

	public:
		boost::filesystem::path getFilePath() const { return filePath; }
		PTR(OCT) getOCT() const { return oct; }

	protected:
		boost::filesystem::path filePath;
		PTR(OCT) oct;

		virtual void LoadMeta();
		virtual void DumpMeta() const;
	};
}

#include "ContainerPcNDFOC.hpp"
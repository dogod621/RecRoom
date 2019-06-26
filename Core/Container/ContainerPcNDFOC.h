#pragma once

#include <pcl/outofcore/outofcore.h>
#include <pcl/outofcore/outofcore_impl.h>

#include "ContainerPcNDF.h"

namespace RecRoom
{
	class ContainerPcNDFOC : public ContainerPcNDF, public DumpAble
	{
	public:
		using OCTDC = pcl::outofcore::OutofcoreOctreeDiskContainer<PointNDF>;
		using OCT = pcl::outofcore::OutofcoreOctreeBase<OCTDC, PointNDF>;

	public:
		ContainerPcNDFOC(const boost::filesystem::path& filePath);

	public:
		virtual void Merge(const PTR(PcNDF)& v);
		virtual std::size_t Size() const { return size; }
		virtual PTR(PcNDF) Quary(std::size_t i) const;

	public:
		PTR(OCT) getOCT() const { return oct; }

	protected:
		PTR(OCT) oct;
		std::size_t size;

		virtual void Load() { DumpAble::Load(); };
		virtual void Dump() const { DumpAble::Dump(); };
		virtual void Load(const nlohmann::json& j);
		virtual void Dump(nlohmann::json& j) const;
		virtual bool CheckNew() const;
	};
}

#include "ContainerPcNDFOC.hpp"
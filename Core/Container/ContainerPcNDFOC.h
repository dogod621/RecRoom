#pragma once

#include <pcl/outofcore/outofcore.h>
#include <pcl/outofcore/outofcore_impl.h>

#include "ContainerPcNDF.h"

namespace RecRoom
{
	class ContainerPcNDFOC : public ContainerPcNDF, public DumpAble
	{
	public:
		using OCT = pcl::outofcore::OutofcoreOctreeBase<pcl::outofcore::OutofcoreOctreeDiskContainer<PointNDF>, PointNDF>;
		using Data = ContainerPcNDF::Data;

	public:
		ContainerPcNDFOC(const boost::filesystem::path& filePath);

	public:
		virtual void Merge(const CONST_PTR(PcNDF)& v);
		virtual std::size_t NumLabel() const { return numLabel; }
		virtual std::size_t NumSerialNumber() const { return numSerialNumber; }
		virtual Data GetData(std::size_t label, std::size_t serialNumber) const;
		virtual Data GetData(std::size_t label) const;

	public:
		PTR(OCT) getOCT() const { return oct; }

	protected:
		PTR(OCT) oct;
		std::size_t numLabel;
		std::size_t numSerialNumber;

		virtual void Load() { DumpAble::Load(); };
		virtual void Dump() const { DumpAble::Dump(); };
		virtual void Load(const nlohmann::json& j);
		virtual void Dump(nlohmann::json& j) const;
		virtual bool CheckExist() const;
	};
}

#include "ContainerPcNDFOC.hpp"
#pragma once

#include <pcl/outofcore/outofcore.h>
#include <pcl/outofcore/outofcore_impl.h>

#include "ContainerPcRAW.h"

namespace RecRoom
{
	class ContainerPcRAWOC : public ContainerPcRAW, public DumpAble
	{
	public:
		using OCTDC = pcl::outofcore::OutofcoreOctreeDiskContainer<PointRAW>;
		using OCT = pcl::outofcore::OutofcoreOctreeBase<OCTDC, PointRAW>;
		using QuaryMeta = ContainerPcRAW::QuaryMeta;
		using QuaryData = ContainerPcRAW::QuaryData;

	public:
		ContainerPcRAWOC(const boost::filesystem::path& filePath,
			const Eigen::Vector3d& min = Eigen::Vector3d(-20.0, -20.0, -20.0),
			const Eigen::Vector3d& max = Eigen::Vector3d(20.0, 20.0, 20.0),
			const double res = 2, double overlap = 0.1);

	public:
		virtual void Merge(const PTR(PcRAW)& v);
		virtual std::size_t Size() const { return quaries.size(); };
		virtual QuaryData Quary(std::size_t i) const;
		virtual QuaryMeta TestQuary(std::size_t i) const;

	public:
		PTR(OCT) getOCT() const { return oct; }
		double getOverlap() const { return overlap; }

	protected:
		PTR(OCT) oct;
		double overlap;
		std::vector<QuaryMeta> quaries;

		virtual void Load() { DumpAble::Load(); };
		virtual void Dump() const { DumpAble::Dump(); };
		virtual void Load(const nlohmann::json& j);
		virtual void Dump(nlohmann::json& j) const;
		virtual bool CheckNew() const;
	};
}

#include "ContainerPcRAWOC.hpp"
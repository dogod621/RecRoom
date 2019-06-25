#pragma once

#include <pcl/outofcore/outofcore.h>
#include <pcl/outofcore/outofcore_impl.h>

#include "ContainerPcRAW.h"

namespace RecRoom
{
	class ContainerPcRAWOC : public ContainerPcRAW
	{
	public:
		using OCTDC = pcl::outofcore::OutofcoreOctreeDiskContainer<PointRAW>;
		using OCT = pcl::outofcore::OutofcoreOctreeBase<OCTDC, PointRAW>;
		using QuaryMeta = ContainerPcRAW::QuaryMeta;
		using QuaryData = ContainerPcRAW::QuaryData;

	public:
		ContainerPcRAWOC(const boost::filesystem::path& filePath);

		ContainerPcRAWOC(const boost::filesystem::path& filePath, const Eigen::Vector3d& min, const Eigen::Vector3d& max, const double res, double overlap);

	public:
		virtual void Merge(const PTR(PcRAW)& v);
		virtual std::size_t Size() const { return quaries.size(); };
		virtual QuaryData Quary(std::size_t i) const;
		virtual QuaryMeta TestQuary(std::size_t i) const;

	public:
		boost::filesystem::path getFilePath() const { return filePath; }
		PTR(OCT) getOCT() const { return oct; }
		double getOverlap() const { return overlap; }

	protected:
		boost::filesystem::path filePath;
		PTR(OCT) oct;
		double overlap;
		std::vector<QuaryMeta> quaries;

		virtual void LoadMeta();
		virtual void DumpMeta() const;
	};
}

#include "ContainerPcRAWOC.hpp"
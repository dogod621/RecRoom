#pragma once

#include <pcl/outofcore/outofcore.h>
#include <pcl/outofcore/outofcore_impl.h>

#include "ContainerPcRAW.h"

namespace RecRoom
{
	struct QuaryPcRAWOCT
	{
		Eigen::Vector3d minAABB;
		Eigen::Vector3d maxAABB;
		Eigen::Vector3d extMinAABB;
		Eigen::Vector3d extMaxAABB;
		std::size_t depth;

		QuaryPcRAWOCT(
			const Eigen::Vector3d& minAABB = Eigen::Vector3d(0.0, 0.0, 0.0),
			const Eigen::Vector3d& maxAABB = Eigen::Vector3d(0.0, 0.0, 0.0),
			const Eigen::Vector3d& extMinAABB = Eigen::Vector3d(0.0, 0.0, 0.0),
			const Eigen::Vector3d& extMaxAABB = Eigen::Vector3d(0.0, 0.0, 0.0),
			std::size_t depth = 0)
			: minAABB(minAABB), maxAABB(maxAABB), extMinAABB(extMinAABB), extMaxAABB(extMaxAABB), depth(depth) {}
	};

	class ContainerPcRAWOCT : public ContainerPcRAW
	{
	public:
		using OCTDC = pcl::outofcore::OutofcoreOctreeDiskContainer<PointRAW>;
		using OCT = pcl::outofcore::OutofcoreOctreeBase<OCTDC, PointRAW>;

	public:
		ContainerPcRAWOCT(const boost::filesystem::path& filePath);

		ContainerPcRAWOCT(const boost::filesystem::path& filePath, const Eigen::Vector3d& min, const Eigen::Vector3d& max, const double res, double outOfCoreOverlapSize);

	public:
		virtual void Merge(const PTR(PcRAW)& v);
		virtual std::size_t Size() const { return quaries.size(); };
		virtual QuaryPcRAW Quary(std::size_t i) const;

	protected:
		boost::filesystem::path filePath;
		PTR(OCT) oct;
		double outOfCoreOverlapSize;
		std::vector<QuaryPcRAWOCT> quaries;
	};
}

#include "ContainerPcRAWOCT.hpp"
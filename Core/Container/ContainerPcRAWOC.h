#pragma once

#include <pcl/outofcore/outofcore.h>
#include <pcl/outofcore/outofcore_impl.h>

#include "ContainerPcRAW.h"

namespace RecRoom
{
	class ContainerPcRAWOC : public ContainerPcRAW, public DumpAble
	{
	public:
		using OCT = pcl::outofcore::OutofcoreOctreeBase<pcl::outofcore::OutofcoreOctreeDiskContainer<PointRAW>, PointRAW>;
		using Meta = ContainerPcRAW::Meta;
		using Data = ContainerPcRAW::Data;

	public:
		ContainerPcRAWOC(const boost::filesystem::path& filePath,
			const Eigen::Vector3d& min = Eigen::Vector3d(-20.0, -20.0, -20.0),
			const Eigen::Vector3d& max = Eigen::Vector3d(20.0, 20.0, 20.0),
			const double res = 2, double overlap = 0.1);

		virtual Eigen::Vector3d getMinAABB() const
		{
			Eigen::Vector3d min; 
			Eigen::Vector3d max;
			oct->getBoundingBox(min, max);
			return min;
		}

		virtual Eigen::Vector3d getMaxAABB() const
		{
			Eigen::Vector3d min;
			Eigen::Vector3d max;
			oct->getBoundingBox(min, max);
			return max;
		}

	public:
		virtual void Merge(const CONST_PTR(PcRAW)& v);
		virtual std::size_t Size() const { return metaSet.size(); };
		virtual Meta GetMeta(std::size_t i) const;
		virtual Data GetData(std::size_t i) const;
		
	public:
		PTR(OCT) getOCT() const { return oct; }
		double getOverlap() const { return overlap; }

	protected:
		PTR(OCT) oct;
		double overlap;
		std::vector<Meta> metaSet;

		virtual void Load() { DumpAble::Load(); };
		virtual void Dump() const { DumpAble::Dump(); };
		virtual void Load(const nlohmann::json& j);
		virtual void Dump(nlohmann::json& j) const;
		virtual bool CheckExist() const;

	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};
}

#include "ContainerPcRAWOC.hpp"
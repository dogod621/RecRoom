#pragma once

#include "Common/Common.h"
#include "Common/Point.h"

namespace RecRoom
{
	class ContainerPcRAW
	{
	public:
		struct Meta
		{
			Eigen::Vector3d minAABB;
			Eigen::Vector3d maxAABB;
			Eigen::Vector3d extMinAABB;
			Eigen::Vector3d extMaxAABB;
			std::size_t depth;

			Meta(
				const Eigen::Vector3d& minAABB = Eigen::Vector3d(0.0, 0.0, 0.0),
				const Eigen::Vector3d& maxAABB = Eigen::Vector3d(0.0, 0.0, 0.0),
				const Eigen::Vector3d& extMinAABB = Eigen::Vector3d(0.0, 0.0, 0.0),
				const Eigen::Vector3d& extMaxAABB = Eigen::Vector3d(0.0, 0.0, 0.0),
				std::size_t depth = 0)
				: minAABB(minAABB), maxAABB(maxAABB), extMinAABB(extMinAABB), extMaxAABB(extMaxAABB), depth(depth) {}

			EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		};

		struct Data : public Meta
		{
			PTR(PcMED) pcMED;
			PTR(PcIndex) pcIndex;

			Data(const Meta& meta = Meta())
				: Meta(meta),
				pcMED(new PcMED), pcIndex(new PcIndex) {}
		};

	public:
		ContainerPcRAW() {}

		virtual Eigen::Vector3d getMinAABB() const = 0;
		virtual Eigen::Vector3d getMaxAABB() const = 0;

	public:
		virtual void Merge(const CONST_PTR(PcRAW)& v) = 0;
		virtual std::size_t Size() const = 0;
		virtual Meta GetMeta(std::size_t i) const = 0;
		virtual Data GetData(std::size_t i) const = 0;
	};
}

#include "ContainerPcRAW.hpp"
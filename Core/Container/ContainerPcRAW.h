#pragma once

#include "Common/Common.h"
#include "Common/Point.h"

namespace RecRoom
{
	class ContainerPcRAW
	{
	public:
		struct QuaryMeta
		{
			Eigen::Vector3d minAABB;
			Eigen::Vector3d maxAABB;

			QuaryMeta() : minAABB(Eigen::Vector3d(0.0, 0.0, 0.0)), maxAABB(Eigen::Vector3d(0.0, 0.0, 0.0)) {}
		};

		struct QuaryData : public QuaryMeta
		{
			PTR(PcMED) data;
			PTR(PcIndex) index;

			QuaryData() : QuaryMeta(), data(new PcMED), index(new PcIndex) {}
		};

	public:
		ContainerPcRAW() {}

	public:
		virtual void Merge(const PTR(PcRAW)& v) = 0;
		virtual std::size_t Size() const = 0;
		virtual QuaryData Quary(std::size_t i) const = 0;
		virtual QuaryMeta TestQuary(std::size_t i) const = 0;
	};
}

#include "ContainerPcRAW.hpp"
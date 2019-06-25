#pragma once

#include "Common/Common.h"
#include "Common/Point.h"

namespace RecRoom
{
	class ContainerPcLF
	{
	public:
		struct QuaryMeta
		{
			/*Eigen::Vector3d minAABB;
			Eigen::Vector3d maxAABB;
			Eigen::Vector3d extMinAABB;
			Eigen::Vector3d extMaxAABB;
			std::size_t depth;

			QuaryMeta(
				const Eigen::Vector3d& minAABB = Eigen::Vector3d(0.0, 0.0, 0.0),
				const Eigen::Vector3d& maxAABB = Eigen::Vector3d(0.0, 0.0, 0.0),
				const Eigen::Vector3d& extMinAABB = Eigen::Vector3d(0.0, 0.0, 0.0),
				const Eigen::Vector3d& extMaxAABB = Eigen::Vector3d(0.0, 0.0, 0.0),
				std::size_t depth = 0)
				: minAABB(minAABB), maxAABB(maxAABB), extMinAABB(extMinAABB), extMaxAABB(extMaxAABB), depth(depth) {}*/
		};

		struct QuaryData : public QuaryMeta
		{
			/*PTR(PcMED) data;
			PTR(PcIndex) index;

			QuaryData(const QuaryMeta& quaryMeta = QuaryMeta()) 
				: QuaryMeta(quaryMeta),
				data(new PcMED), index(new PcIndex) {}*/
		};

	public:
		ContainerPcLF() {}

	public:
		virtual void Merge(const PTR(PcLF)& v) = 0;
		virtual std::size_t Size() const = 0;
		virtual QuaryData Quary(std::size_t i) const = 0;
		virtual QuaryMeta TestQuary(std::size_t i) const = 0;
	};
}

#include "ContainerPcLF.hpp"
#pragma once

#include "FilterPc.h"

namespace RecRoom
{
	template<class PointType>
	class FilterPcAABB : public FilterPc<PointType>
	{
	public:
		FilterPcAABB(
			const Eigen::Vector3d& minAABB = Eigen::Vector3d(0.0, 0.0, 0.0),
			const Eigen::Vector3d& maxAABB = Eigen::Vector3d(0.0, 0.0, 0.0)) 
			: FilterPc<PointType>(), minAABB(minAABB), maxAABB(maxAABB) {}

	protected:
		virtual bool ImplementCheck(
			const CONST_PTR(Acc<PointType>)& searchSurface,
			const CONST_PTR(Pc<PointType>)& input,
			const CONST_PTR(PcIndex)& filter,
			PcIndex& output) const
		{
			if (searchSurface)
				PRINT_WARNING("searchSurface is not used");
			return true;
		}

		virtual void ImplementProcess(
			const CONST_PTR(Acc<PointType>)& searchSurface,
			const CONST_PTR(Pc<PointType>)& input,
			const CONST_PTR(PcIndex)& filter,
			PcIndex& output) const;

	public:
		Eigen::Vector3d getMinAABB() const { return minAABB; }
		Eigen::Vector3d getMaxAABB() const { return maxAABB; }
		void setMinAABB(const Eigen::Vector3d& v) { minAABB = v; }
		void setMaxAABB(const Eigen::Vector3d& v) { maxAABB = v; }

	protected:
		Eigen::Vector3d minAABB;
		Eigen::Vector3d maxAABB;
	};
}

#include "FilterPcAABB.hpp"
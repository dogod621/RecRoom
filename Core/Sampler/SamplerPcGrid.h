#pragma once

#include "SamplerPc.h"

namespace RecRoom
{
	template<class PointType>
	class SamplerPcGrid : public SamplerPc<PointType>
	{
	public:
		SamplerPcGrid(double voxelSize, const Eigen::Vector3d& minAABB, const Eigen::Vector3d& maxAABB, double tooCloseRatio = 0.5)
			: SamplerPc<PointType>(), voxelSize(voxelSize), minAABB(minAABB), maxAABB(maxAABB), tooCloseRatio(tooCloseRatio)
		{
			if ((tooCloseRatio < 0.0) or (tooCloseRatio >= 1.0))
				THROW_EXCEPTION("tooCloseRatio must >=0.0 and < 1.0");
		}

	protected:
		virtual bool ImplementCheck(
			const CONST_PTR(Acc<PointType>)& searchSurface,
			const CONST_PTR(Pc<PointType>)& input,
			const CONST_PTR(PcIndex)& filter,
			Pc<PointType>& output) const 
		{
			if (searchSurface)
				PRINT_WARNING("searchSurface is not used");
			return true;
		}

		virtual void ImplementProcess(
			const CONST_PTR(Acc<PointType>)& searchSurface,
			const CONST_PTR(Pc<PointType>)& input,
			const CONST_PTR(PcIndex)& filter,
			Pc<PointType>& output) const;

	public:
		double getVoxelSize() const { return voxelSize; }
		Eigen::Vector3d getMinAABB() const { return minAABB; }
		Eigen::Vector3d getMaxAABB() const { return maxAABB; }
		double getTooCloseRatio() const { return tooCloseRatio; }

		void setVoxelSize(double v) { voxelSize = v; }
		void setMinAABB(const Eigen::Vector3d& v ) { minAABB = v; }
		void setMaxAABB(const Eigen::Vector3d& v ) { maxAABB = v; }
		void setTooCloseRatio(double v)
		{ 
			if ((v < 0.0) or (v >= 1.0))
			{
				THROW_EXCEPTION("tooCloseRatio must >=0.0 and < 1.0");
			}
			else
				tooCloseRatio = v;
		}

	protected:
		Eigen::Vector3d minAABB;
		Eigen::Vector3d maxAABB;
		double voxelSize;
		double tooCloseRatio;
	};
}

#include "SamplerPcGrid.hpp"
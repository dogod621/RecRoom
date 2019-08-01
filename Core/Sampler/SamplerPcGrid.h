#pragma once

#include "Common/VoxelGrid.h"
#include "SamplerPc.h"

namespace RecRoom
{
	template<class PointType>
	class SamplerPcGrid : public SamplerPc<PointType>
	{
	public:
		using Interpolator = SamplerPc<PointType>::Interpolator;
		using InterpolatorNearest = SamplerPc<PointType>::InterpolatorNearest;

	public:
		SamplerPcGrid(
			double voxelSize, const Eigen::Vector3d& minAABB, const Eigen::Vector3d& maxAABB, double tooCloseRatio = 0.5,
			CONST_PTR(Interpolator) fieldInterpolator = CONST_PTR(Interpolator)(new InterpolatorNearest))
			: SamplerPc<PointType>(fieldInterpolator), voxelSize(voxelSize), minAABB(minAABB), maxAABB(maxAABB), tooCloseRatio(tooCloseRatio)
		{
			name = "SamplerPcGrid";

			if ( tooCloseRatio >= 1.0 )
				THROW_EXCEPTION("tooCloseRatio must < 1.0");
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
		inline virtual bool SearchPointValid(const PointType& p) const
		{
			PRINT_WARNING("not used");
			return true;
		}

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
			if (v >= 1.0)
			{
				THROW_EXCEPTION("tooCloseRatio must < 1.0");
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

	template<class PointType>
	class SamplerPcBinaryGrid : public SamplerPcGrid<PointType>
	{
	public:
		using Interpolator = SamplerPcGrid<PointType>::Interpolator;
		using InterpolatorNearest = SamplerPcGrid<PointType>::InterpolatorNearest;

	public:
		SamplerPcBinaryGrid(double voxelSize, const Eigen::Vector3d& minAABB, const Eigen::Vector3d& maxAABB,
			MorphologyOperation morphologyOperation = MorphologyOperation::MorphologyOperation_NONE, std::size_t kernelSize = 0, std::size_t iteration = 0,
			CONST_PTR(Interpolator) fieldInterpolator = CONST_PTR(Interpolator)(new InterpolatorNearest))
			: SamplerPcGrid<PointType>(voxelSize, minAABB, maxAABB, -1.0, fieldInterpolator), morphologyOperation(morphologyOperation), kernelSize(kernelSize), iteration(iteration)
		{
			name = "SamplerPcBinaryGrid";
		}

	protected:
		virtual void ImplementProcess(
			const CONST_PTR(Acc<PointType>)& searchSurface,
			const CONST_PTR(Pc<PointType>)& input,
			const CONST_PTR(PcIndex)& filter,
			Pc<PointType>& output) const;

	public:
		inline virtual bool InputPointValid(const PointType& p) const
		{
			return pcl_isfinite(p.x) &&
				pcl_isfinite(p.y) &&
				pcl_isfinite(p.z);
		}

		inline virtual bool OutPointValid(const PointType& p) const
		{
			return pcl_isfinite(p.x) &&
				pcl_isfinite(p.y) &&
				pcl_isfinite(p.z);
		}

	public:
		MorphologyOperation getMorphologyOperation() const { return morphologyOperation; }
		std::size_t getKernelSize() const { return kernelSize; }
		std::size_t getIteration() const { return iteration; }

		void setMorphologyOperation(MorphologyOperation v) { morphologyOperation = v; }
		void setKernelSize(std::size_t v) { kernelSize = v; }
		void setIteration(std::size_t v) { iteration = v; }

	protected:
		MorphologyOperation morphologyOperation;
		std::size_t kernelSize;
		std::size_t iteration;
	};
}

#include "SamplerPcGrid.hpp"
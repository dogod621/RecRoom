#pragma once

#include "SamplerPc.h"

namespace RecRoom
{
	template<class PointType>
	class SamplerPcGrid : public SamplerPc<PointType>
	{
	public:
		SamplerPcGrid(float voxelSize, float tooCloseRatio = 0.5f) 
			: SamplerPc<PointType>(), voxelSize(voxelSize), tooCloseRatio(tooCloseRatio)
		{
			if ((tooCloseRatio < 0.0f) or (tooCloseRatio >= 1.0f))
				THROW_EXCEPTION("tooCloseRatio must >=0.0f and < 1.0f");
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
		float getVoxelSize() const { return voxelSize; }
		float getTooCloseRatio() const { return tooCloseRatio; }
		void setVoxelSize(float v) { voxelSize = v; }
		void setTooCloseRatio(float v) 
		{ 
			if ((v < 0.0f) or (v >= 1.0f))
			{
				THROW_EXCEPTION("tooCloseRatio must >=0.0f and < 1.0f");
			}
			else
				tooCloseRatio = v;
		}

	protected:
		float voxelSize;
		float tooCloseRatio;
	};
}

#include "SamplerPcGrid.hpp"
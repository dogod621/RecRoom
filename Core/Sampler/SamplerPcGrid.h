#pragma once

#include "SamplerPc.h"

namespace RecRoom
{
	template<class PointType>
	class SamplerPcGrid : public ResamplerPc<PointType>
	{
	public:
		SamplerPcGrid(float voxelSize) : ResamplerPc<PointType>(), voxelSize(voxelSize){}

	public:
		virtual void Process(
			const PTR(Pc<PointType>) & inV,
			Pc<PointType> & outV) const;

	public:
		float getVoxelSize() const { return voxelSize; }
		void setVoxelSize(float v) { voxelSize = v; }

	protected:
		float voxelSize;
	};

	using SamplerPcGridRAW = SamplerPcGrid<PointRAW>;
	using SamplerPcGridMED = SamplerPcGrid<PointMED>;
	using SamplerPcGridREC = SamplerPcGrid<PointREC>;
	using SamplerPcGridNDF = SamplerPcGrid<PointNDF>;
	using SamplerPcGridLF = SamplerPcGrid<PointLF>;
}

#include "SamplerPcGrid.hpp"
#pragma once

#include "SamplerPc.h"

namespace RecRoom
{
	class SamplerPcGrid : public ResamplerPc
	{
	public:
		SamplerPcGrid(float voxelSize) : ResamplerPc(), voxelSize(voxelSize){}

	public:
		virtual void Process(
			const PTR(PcMED) & inV,
			PcMED & outV) const;

	public:
		float getVoxelSize() const { return voxelSize; }
		void setVoxelSize(float v) { voxelSize = v; }

	protected:
		float voxelSize;
	};
}

#include "SamplerPcGrid.hpp"
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

	protected:
		float voxelSize;
	};
}

#include "SamplerPcGrid.hpp"
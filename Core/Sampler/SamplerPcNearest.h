#pragma once

#include "SamplerPc.h"

namespace RecRoom
{
	template<class PointType>
	class SamplerPcNearest : public SamplerPc<PointType>
	{
	public:
		SamplerPcNearest() : SamplerPc<PointType>() {}

	public:
		virtual void Process(
			const PTR(Acc<PointType>)& searchMethod,
			const PTR(Pc<PointType>)& searchSurface,
			const PTR(Pc<PointType>)& inV,
			PcIndex& outV) const;
	};

	using SamplerPcNearestRAW = SamplerPcNearest<PointRAW>;
	using SamplerPcNearestMED = SamplerPcNearest<PointMED>;
	using SamplerPcNearestREC = SamplerPcNearest<PointREC>;
	using SamplerPcNearestNDF = SamplerPcNearest<PointNDF>;
	using SamplerPcNearestLF = SamplerPcNearest<PointLF>;
}

#include "SamplerPcNearest.hpp"
#pragma once

#include "Common/Common.h"
#include "Common/Point.h"

namespace RecRoom
{
	template<class PointType>
	class ResamplerPc
	{
	public:
		ResamplerPc() {}

	public:
		virtual void Process(
			const PTR(Pc<PointType>) & inV,
			Pc<PointType> & outV) const = 0;
	};

	template<class PointType>
	class SamplerPc
	{
	public:
		SamplerPc() {}

	public:
		virtual void Process(
			const PTR(Acc<PointType>)& searchMethod,
			const PTR(Pc<PointType>)& searchSurface,
			const PTR(Pc<PointType>)& inV,
			PcIndex& outV) const = 0;
	};

	using ResamplerPcRAW = ResamplerPc<PointRAW>;
	using ResamplerPcMED = ResamplerPc<PointMED>;
	using ResamplerPcREC = ResamplerPc<PointREC>;
	using ResamplerPcNDF = ResamplerPc<PointNDF>;
	using ResamplerPcLF = ResamplerPc<PointLF>;

	using SamplerPcRAW = SamplerPc<PointRAW>;
	using SamplerPcMED = SamplerPc<PointMED>;
	using SamplerPcREC = SamplerPc<PointREC>;
	using SamplerPcNDF = SamplerPc<PointNDF>;
	using SamplerPcLF = SamplerPc<PointLF>;
}

#include "SamplerPc.hpp"
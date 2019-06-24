#pragma once

#include "SamplerPc.h"

namespace RecRoom
{
	class SamplerPcNearest : public SamplerPc
	{
	public:
		SamplerPcNearest() : SamplerPc() {}

	public:
		virtual void Process(
			const PTR(AccMED)& searchMethod,
			const PTR(PcMED)& searchSurface,
			const PTR(PcMED)& inV,
			PcIndex& outV) const;
	};
}

#include "SamplerPcNearest.hpp"
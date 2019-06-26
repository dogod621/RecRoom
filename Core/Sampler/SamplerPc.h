#pragma once

#include "Common/Common.h"
#include "Common/Point.h"

namespace RecRoom
{
	class ResamplerPc
	{
	public:
		ResamplerPc() {}

	public:
		virtual void Process(
			const PTR(PcMED) & inV,
			PcMED & outV) const = 0;
	};

	class SamplerPc
	{
	public:
		SamplerPc() {}

	public:
		virtual void Process(
			const PTR(AccMED)& searchMethod,
			const PTR(PcMED)& searchSurface,
			const PTR(PcMED)& inV) const;

		virtual void Process(
			const PTR(AccMED)& searchMethod,
			const PTR(PcMED)& searchSurface,
			const PTR(PcMED)& inV,
			PcIndex& outV) const = 0;
	};
}

#include "SamplerPc.hpp"
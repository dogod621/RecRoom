#pragma once

#include "Common/Common.h"
#include "Common/Point.h"

namespace RecRoom
{
	class PreprocessorPc
	{
	public:
		PreprocessorPc(double searchRadius)
			: searchRadius(searchRadius) {}

	public:
		virtual void Process(
			const PTR(AccRAW)& searchMethod,
			const PTR(PcRAW)& searchSurface,
			const PTR(PcRAW)& inV,
			const PTR(PcIndex)& inIdx,
			PcRAW& outV) const = 0;

	public:
		double getSearchRadius() const { return searchRadius; }

	protected:
		double searchRadius;
	};
}

#include "PreprocessorPc.hpp"
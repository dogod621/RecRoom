#pragma once

#include "Common/Common.h"
#include "Common/Point.h"

namespace RecRoom
{
	class EstimatorPc
	{
	public:
		EstimatorPc(double searchRadius)
			: searchRadius(searchRadius) {}

	public:
		virtual void Process(
			const PTR(AccMED)& searchMethod,
			const PTR(PcMED)& searchSurface,
			const PTR(PcMED)& inV,
			const PTR(PcIndex)& inIdx,
			PcMED& outV) const = 0;

	public:
		double getSearchRadius() const { return searchRadius; }

	protected:
		double searchRadius;
	};
}

#include "EstimatorPc.hpp"
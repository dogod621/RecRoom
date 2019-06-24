#pragma once

#include "EstimatorPc.h"

namespace RecRoom
{
	class EstimatorPcAlbedo : public EstimatorPc
	{
	public:
		EstimatorPcAlbedo(double searchRadius)
			: EstimatorPc(searchRadius) {}

	public:
		virtual void Process(
			const PTR(AccMED)& searchMethod,
			const PTR(PcMED)& searchSurface,
			const PTR(PcMED)& inV,
			const PTR(PcIndex)& inIdx,
			PcMED& outV) const;
	};
}

#include "EstimatorPcAlbedo.hpp"
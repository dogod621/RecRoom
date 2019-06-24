#pragma once

#include "CropperPc.h"

namespace RecRoom
{
	class CropperPcOutlier : public CropperPc
	{
	public:
		CropperPcOutlier(int meanK=50, double stdMul=1.0) : CropperPc(), meanK(meanK), stdMul(stdMul){}

	public:
		virtual void Process(const PTR(PcMED)& inV, PcIndex& outV) const;

	protected:
		int meanK;
		double stdMul;
	};
}

#include "CropperPcOutlier.hpp"
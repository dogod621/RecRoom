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

	public:
		int getMeanK() const { return meanK; }
		double getStdMul() const { return stdMul; }
		void setMeanK(int v) { meanK = v; }
		void setStdMul(double v) { stdMul = v; }

	protected:
		int meanK;
		double stdMul;
	};
}

#include "CropperPcOutlier.hpp"
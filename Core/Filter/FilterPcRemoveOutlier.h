#pragma once

#include "FilterPc.h"

namespace RecRoom
{
	template<class PointType>
	class FilterPcRemoveOutlier : public FilterPc<PointType>
	{
	public:
		FilterPcRemoveOutlier(int meanK=50, double stdMul=1.0) : FilterPc<PointType>(), meanK(meanK), stdMul(stdMul)
		{
			name = "FilterPcRemoveOutlier";
		}

	public:
		virtual void ImplementProcess(
			const CONST_PTR(Acc<PointType>)& searchSurface,
			const CONST_PTR(Pc<PointType>)& input,
			const CONST_PTR(PcIndex)& filter,
			PcIndex& output) const;

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

#include "FilterPcRemoveOutlier.hpp"
#pragma once

#include "InterpolatorPc.h"

namespace RecRoom
{
	template<class InPointType, class OutPointType>
	class InterpolatorPcNearest : public InterpolatorPc<InPointType, OutPointType>, public ThreadAble
	{
	public:
		InterpolatorPcNearest() : ThreadAble(), InterpolatorPc<InPointType, OutPointType>()
		{
			name = "InterpolatorPcNearest";
		}

	protected:
		virtual void ImplementProcess(
			const CONST_PTR(Acc<InPointType>)& searchSurface,
			const CONST_PTR(Pc<InPointType>)& input,
			const CONST_PTR(PcIndex)& filter,
			Pc<OutPointType>& output) const;
	};
}

#include "InterpolatorPcNearest.hpp"
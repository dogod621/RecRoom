#pragma once

#include "Common/Common.h"
#include "Common/Processor.h"

namespace RecRoom
{
	template<class InPointType, class OutPointType>
	class InterpolatorPc 
		: public SearchAnySurfaceProcessorPc2PcInOut<InPointType, OutPointType>
	{
	public:
		InterpolatorPc() 
			: SearchAnySurfaceProcessorPc2PcInOut<InPointType, OutPointType>()
		{
			name = "InterpolatorPc";
		}
	};
}

#include "InterpolatorPc.hpp"
#pragma once

#include "Common/Common.h"
#include "Common/Processor.h"

namespace RecRoom
{
	template<class InPointType, class OutPointType>
	class InterpolatorPc 
		: public SearchAnySurfaceProcessorPc2Pc<InPointType, OutPointType>
	{
	public:
		InterpolatorPc() 
			: SearchAnySurfaceProcessorPc2Pc<InPointType, OutPointType>() {}
	};
}

#include "InterpolatorPc.hpp"
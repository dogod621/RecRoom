#pragma once

#include "Common/Common.h"

namespace RecRoom
{
	template<class InPointType, class OutPointType>
	class InterpolatorPc 
		: public SearchAnySurfaceProcesserPc2Pc<InPointType, OutPointType>
	{
	public:
		InterpolatorPc() 
			: SearchAnySurfaceProcesserPc2Pc<InPointType, OutPointType>() {}
	};
}

#include "InterpolatorPc.hpp"
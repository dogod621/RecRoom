#pragma once

#include "Common/Common.h"
#include "Common/Processor.h"

namespace RecRoom
{
	template<class PointType>
	class SamplerPc : public SearchInputSurfaceProcessorPc2Pc<PointType, PointType>
	{
	public:
		SamplerPc() : SearchInputSurfaceProcessorPc2Pc<PointType, PointType>() 
		{
			name = "SamplerPc";
		}
	};
}

#include "SamplerPc.hpp"
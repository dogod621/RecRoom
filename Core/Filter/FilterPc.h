#pragma once

#include "Common/Common.h"
#include "Common/Processor.h"

namespace RecRoom
{
	template<class PointType>
	class FilterPc : public SearchInputSurfaceProcessorPc<PointType, PcIndex>
	{
	public:
		FilterPc() : SearchInputSurfaceProcessorPc<PointType, PcIndex>() {}
	};
}

#include "FilterPc.hpp"
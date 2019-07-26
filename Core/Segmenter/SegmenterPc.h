#pragma once

#include "Common/Common.h"
#include "Common/Processor.h"

namespace RecRoom
{
	template<class PointType>
	class SegmenterPc 
		: public SearchInputSurfaceProcessorPc2Pc<PointType, PointType>
	{
	public:
		SegmenterPc() 
			: SearchInputSurfaceProcessorPc2Pc<PointType, PointType>() 
		{
			name = "SegmenterPc";
		}
	};
}

#include "SegmenterPc.hpp"
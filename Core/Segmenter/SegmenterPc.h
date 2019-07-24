#pragma once

#include "Common/Common.h"

namespace RecRoom
{
	template<class PointType>
	class SegmenterPc 
		: public SearchInputSurfaceProcesserPc2Pc<PointType, PointType>
	{
	public:
		SegmenterPc() 
			: SearchInputSurfaceProcesserPc2Pc<PointType, PointType>() {}
	};
}

#include "SegmenterPc.hpp"
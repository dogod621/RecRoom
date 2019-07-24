#pragma once

#include "Common/Common.h"

namespace RecRoom
{
	template<class PointType>
	class FilterPc : public SearchInputSurfaceProcesserPc<PointType, PcIndex>
	{
	public:
		FilterPc() : SearchInputSurfaceProcesserPc<PointType, PcIndex>() {}
	};
}

#include "FilterPc.hpp"
#pragma once

#include "Common/Common.h"

namespace RecRoom
{
	template<class PointType>
	class SamplerPc : public SearchInputSurfaceProcesserPc2Pc<PointType, PointType>
	{
	public:
		SamplerPc() : SearchInputSurfaceProcesserPc2Pc<PointType, PointType>() {}
	};
}

#include "SamplerPc.hpp"
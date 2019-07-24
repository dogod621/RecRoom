#pragma once

#include "Common/Common.h"
#include "Common/Point.h"

namespace RecRoom
{
	class PreprocessorPc : 
		public SearchInputSurfaceProcesserPc2Pc<PointRAW, PointRAW>
	{
	public:
		PreprocessorPc() : SearchInputSurfaceProcesserPc2Pc<PointRAW, PointRAW>() {}
	};
}

#include "PreprocessorPc.hpp"